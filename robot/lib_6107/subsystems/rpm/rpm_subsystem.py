# ------------------------------------------------------------------------ #
#      o-o      o                o                                         #
#     /         |                |                                         #
#    O     o  o O-o  o-o o-o     |  oo o--o o-o o-o                        #
#     \    |  | |  | |-' |   \   o | | |  |  /   /                         #
#      o-o o--O o-o  o-o o    o-o  o-o-o--O o-o o-o                        #
#             |                           |                                #
#          o--o                        o--o                                #
#                        o--o      o         o                             #
#                        |   |     |         |  o                          #
#                        O-Oo  o-o O-o  o-o -o-    o-o o-o                 #
#                        |  \  | | |  | | |  |  | |     \                  #
#                        o   o o-o o-o  o-o  o  |  o-o o-o                 #
#                                                                          #
#    Jemison High School - Huntsville Alabama                              #
# ------------------------------------------------------------------------ #

import logging
from copy import deepcopy
from enum import Enum, unique
from typing import Any, Callable, Optional, Tuple

from commands2 import Subsystem
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from phoenix6.hardware import TalonFX
from rev import REVLibError, SparkBaseConfig, \
    SparkClosedLoopController, SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim, \
    SparkRelativeEncoder
from wpilib import RobotBase
from wpilib.simulation import RoboRioSim
from wpilib.sysid import SysIdRoutineLog
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, radians, radians_per_second, radiansPerSecondToRotationsPerMinute, \
    revolutions_per_minute, seconds, volts

from lib_6107.pykit.logger import Logger
from lib_6107.subsystems.pykit.rpm_mechanism_io import RpmMechanismIO
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)

SupportedMotors = SparkMax | SparkFlex | TalonFX
SupportedSimMotors = SparkMaxSim | SparkFlexSim
SupportedEncoders = SparkRelativeEncoder
SupportedClosedLoopControllers = SparkClosedLoopController


@unique
class ControllerType(Enum):
    """
    Currently the following motor/controller types are supported
    """
    SparkMax = "SparkMax"       # Rev Robotics
    SparkFlex = "SparkFlex"     # Rev Robotics
    KrakenX60 = "KrakenX60"     # CTRE  - TalonFX

def _default_max_rpm(controller_type: ControllerType, motor: DCMotor) -> revolutions_per_minute:
    # TODO: Future, for when None passed in for MAX_RPM
    return 0.0


class RpmConfig:
    """
    The subclass should provide a set of constants. If any of the 'REQUIRED' constants are
    set to 'None', then these defaults will be inherited
    """
    proportional_coefficient = 0.0001  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0           # kI - If you haven’t been where you want to be for a while, apply more effort
                                       #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0         # kD - If you’re getting close to where you want to be, slow down.
    izone = None                       #      If you are really far from where you want to be, don’t start applying
                                       #      more effort to get there until you are within this margin
    velocity_feedforward = None
    imax_accum = None

    limit_current: amperes = 40

    # Following are optional and do not need to be provided by the
    # derived class unless they need to be overridden
    gear_reduction = 1.0
    measurement_std_dev = [0.0, 0.0]
    max_rpm: Optional[revolutions_per_minute] = None  # Must be set by subclass

    # Following are required in the base class. Derived classes may have more
    __rpm_required_attributes = ("max_rpm", "limit_current", "proportional_coefficient",
                                 "integral_coefficient", "derivative_coefficient")

    @property
    def required_attributes(self) -> Tuple[str, ...]:
        return self.__rpm_required_attributes


#@autologgable_output
class RpmSubsystem(Subsystem, RpmMechanismIO):
    """
    A subsystem with a single motor that typically has an RPM goal.  This will provide a
    base-class that has a PID Controller that has velocity (rotational) goal to achieve
    and maintain.
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool, name: str,
                 controller_type: ControllerType, constants: RpmConfig,
                 long_name: str | None) -> None:
        #
        # Set the following to True at the very end of your actual SubSystem init. This is to
        # avoid a race condition in 'pykit' where the 'periodic' and other functions may be
        # called for a subsystem 'before' full initialization. This is due to how some vendor
        # firmware routines work.
        #
        self._initialized = False
        #
        Subsystem.__init__(self)
        RpmMechanismIO.__init__(self, name)

        # General attributes
        self.setName(name)

        self._controller_type = controller_type
        self._long_name = long_name or name  # Typically for logging/smartdashboard such as "intake/indexer"
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._inputs = RpmMechanismIO.RpmMechanismIOInputs()
        self._is_simulation = RobotBase.isSimulation()

        # Simulation only (set in derived class __init__ after this call or in sim_init)
        self._physics_controller = None
        self._sim_motor: SupportedSimMotors | None = None

        # Derived class sets or are reinitialized these 'after' calling this base class init
        self._motor: SupportedMotors | None = None
        self._constants: RpmConfig = constants

        # Following are defined in the post_init call from the derived class
        self._is_connected: bool = False
        self._pid_controller: SupportedClosedLoopControllers | None = None

        # The critical attributes/properties for operation
        self._velocity_goal: revolutions_per_minute = 0.0
        self._velocity_tolerance: revolutions_per_minute = 0.0

        # SysID Support
        self._sysid_routine = SysIdRoutine(SysIdRoutine.Config(),
                                           SysIdRoutine.Mechanism(lambda voltage: self._set_voltage(voltage),
                                                                  lambda log: self._log_motor(log),
                                                                  self,
                                                                  name))

    def post_init(self, coast: bool, persist_config: bool) -> None:
        # Sanity / defaults check
        self._constants = self._validate_constants()

    def _validate_constants(self) -> Any:
        """
        Validate that the constants passed in have values/properties this class needs. They
        can be None if you want this class to use a default value (often zero), but they do need
        to exist as an explicit attribute of the object passed in
        """
        constants = deepcopy(self._constants)

        for attribute in RpmConfig().required_attributes:
            # Needs to be there, even if set to None
            assert hasattr(constants, attribute), f"{attribute} was not found in {self.getName()} object config"

            # If set to None, use our default values (which may be None as well)
            if getattr(constants, attribute, None) is None:
                setattr(constants, attribute, getattr(RpmConfig(), attribute))

        return constants

    def try_until_ok(self, what: str, attempts: int, command: Callable[[], Any]) -> Any:
        """
        Repeats a command to the underlying motor/controller until success or attempts
        exhausted.
        """
        raise NotImplementedError("try_until_ok: Implement in a derived class")

    def _motor_config(self, coast: bool) -> SparkBaseConfig:
        """
        Motor config for the intake Indexer. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        raise NotImplementedError("_motor_config: Implement in a derived class")

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    def _check_is_connected(self, config_status: REVLibError | None) -> bool:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        raise NotImplementedError("_check_is_connected: Implement in a derived class")

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        raise NotImplementedError("is_connected: Implement in a derived class")

    @property
    def goal(self) -> revolutions_per_minute:
        return self._velocity_goal

    @property
    def tolerance(self) -> revolutions_per_minute:
        return self._velocity_tolerance

    @property
    def velocity_in_rpm(self) -> revolutions_per_minute:
        return radiansPerSecondToRotationsPerMinute(self.velocity_in_rps)

    @property
    def velocity_in_rps(self) -> radians_per_second:
        raise NotImplementedError("velocity_in_rps: Implement in a derived class")

    @property
    def position(self) -> radians:
        raise NotImplementedError("position: Implement in a derived class")

    @property
    def active(self) -> bool:
        """
        True if the goal RPM is non-zero or the mechanism is still spinning
        """
        return self.goal != 0.0 and self.velocity_in_rps != 0.0

    @property
    def not_ready(self) -> str:
        velocity = self.velocity_in_rpm
        if velocity < self.goal - self.tolerance:
            return f"under velocity goal: {velocity} < {self.goal}"

        if velocity > self.goal + self.tolerance:
            return f"above velocity goal: {velocity} > {self.goal}"

        return ""  # indexer is ready (within tolerated limits

    def _set_velocity_goal(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None) -> None:
        raise NotImplementedError("_set_velocity_goal: Implement in a derived class")

    def stop(self) -> None:
        logger.info(f"{self.getName()}: Stop command was called")
        self._set_velocity_goal(0, 0)
        self._motor.disable()

    # TODO: Add periodic support for getting any faults so we can display them back to the user and
    #       possibly clean them on startup if they are sticky. But not every periodic call...

    def periodic(self) -> None:
        if self.is_initialized:
            LogTracer.resetOuter(f"{self.getName()} periodic")

            self.updateInputs(self._inputs)

            Logger.processInputs(self.getName(), self._inputs)
            LogTracer.record("UpdateInputs")

            # TODO: Look what we minimally need to do if we want to provide replay?

            Logger.recordOutput(f"{self._long_name}/goal", self.goal)
            Logger.recordOutput(f"{self._long_name}/current", self.velocity_in_rpm)
            Logger.recordOutput(f"{self._long_name}/tolerance", self.tolerance)
            LogTracer.recordTotal()

            # Update SmartDashboard for this subsystem at a rate slower than the period
            counter = self._robot.counter
            if counter % 100 == 0 or (self._robot.counter % 31 == 0 and
                                      self._robot.isEnabled()):
                self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        # SmartDashboard.putNumber(f"{self._long_name}/Goal",0.0)
        # SmartDashboard.putNumber(f"{self._long_name}/Tolerance", 0.0)
        # SmartDashboard.putNumber(f"{self._long_name}/Current", 0.0)
        # SmartDashboard.putNumber(f"{self._long_name}/Voltage", 0.0)
        # SmartDashboard.putNumber(f"{self._long_name}/Current", 0.0)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        # SmartDashboard.putNumber(f"{self._long_name}/Goal", self.goal)
        # SmartDashboard.putNumber(f"{self._long_name}/Tolerance", self.tolerance)
        # SmartDashboard.putNumber(f"{self._long_name}/Current",  self._inputs.mechanism_velocity)
        # SmartDashboard.putNumber(f"{self._long_name}/Voltage", self._inputs.mechanism_applied_voltage)
        # SmartDashboard.putNumber(f"{self._long_name}/Current", self._inputs.mechanism_supply_current)

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        raise NotImplementedError("updateInputs: Implement in a derived class")

    def fault_detection(self, state: str, clear: Optional[bool] = True, notify: Optional[bool] = True) -> None:
        """
        This routine is responsible for reading any existing faults and based
        input parameters, report them for display, and possibly clear them

        All faults detected always results in a warning log message, so please be
        aware of this if you do not clear them
        """
        raise NotImplementedError("fault_detection: Implement in a derived class")

    ###########################################################
    # Simulation Support

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller

    def update_sim(self, now: float, tm_diff: float) -> amperes | None:
        """
        Called when the simulation parameters for the program need to be updated.
        This function is called from the '_simulationPeriodic' function of the
        robotpy core routine and is called at a period >= 10 mS. Note that the
        CommandScheduler also has an 'simulationPeriodic' function that it calls
        into all Command2 based subsystems at its update period which has a
        default rate of 20 mS.

        This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
        that function uses pykit's logging method, you should use those values in
        your simulation.

        :param now:     The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if self._robot.isEnabled() and self._sim_motor is not None:
            voltage = RoboRioSim.getVInVoltage()
            # logger.info(f"{self.getName()} iterate")
            self._sim_motor.iterate(self.velocity_in_rpm, voltage, tm_diff)

            # And simulate current drain
            return self._sim_motor.getMotorCurrent()
        return None

    ###########################################################
    # SysID Support

    def _log_motor(self, log: SysIdRoutineLog):
        (
            log.motor(self.getName())
            .position(self._encoder.getPosition())
            .velocity(self._encoder.getVelocity())
            .voltage(self._motor.getAppliedOutput() * self._motor.getBusVoltage())
        )

    def _set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        if voltage != self._motor.getAppliedOutput():
            logger.info(f"{self.getName()}: Setting voltage to {voltage}")
            self._motor.setVoltage(voltage)

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Assign this function to either a controller button or add it as a selectable
        Autonomous function to run and then move the robot to Autonomous mode.
        """
        return self._sysid_routine.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Assign this function to either a controller button or add it as a selectable
        Autonomous function to run and then move the robot to Autonomous mode.
        """
        return self._sysid_routine.dynamic(direction)