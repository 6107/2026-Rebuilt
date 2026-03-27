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
import math
from enum import Enum, unique
from typing import Any, Optional

from commands2 import Subsystem
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import ClosedLoopSlot, PersistMode, ResetMode, REVLibError, SparkBase, SparkBaseConfig, \
    SparkClosedLoopController, SparkFlex, SparkFlexConfig, SparkFlexSim, SparkMax, SparkMaxConfig, SparkMaxSim, \
    SparkRelativeEncoder
from wpilib import RobotBase, SmartDashboard
from wpilib.simulation import RoboRioSim
from wpilib.sysid import SysIdRoutineLog
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, radians, radians_per_second, radiansPerSecondToRotationsPerMinute, \
    revolutions_per_minute, seconds, volts

from lib_6107.subsystems.pykit.rpm_mechanism_io import RpmMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


@unique
class ControllerType(Enum):
    """
    Currently the following motor/controller types are supported
    """
    SparkMax = "SparkMax"
    SparkFlex = "SparkFlex"


def _default_max_rpm(controller_type: ControllerType, motor: DCMotor) -> revolutions_per_minute:
    # TODO: Future, for when None passed in for MAX_RPM
    return 0.0


class DefaultConstants:
    PROPORTIONAL_COEFFICIENT = 10  # kP
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0  # kD

    VELOCITY_FEEDFORWARD = None
    IMAX_ACCUM = None
    IZONE = None

    LIMIT_CURRENT: amperes = 40

    # Following optional. Need a way to allow this
    GEAR_REDUCTION = 1.0
    MEASUREMENT_STD_DEV = [0.0, 0.0]
    MAX_RPM: revolutions_per_minute = 6784.0

@autologgable_output
class RpmSubsystem(Subsystem, RpmMechanismIO):
    """
    A subsystem with a single motor that typically has an RPM goal

    This is the rolly-grabber at the front of the intake.
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool, name: str,
                 motor: DCMotor, controller_type: ControllerType, constants: Any,
                 long_name: Optional[str] = None,
                 coast: Optional[bool] = True,
                 persist_config: Optional[bool] = False) -> None:

        Subsystem.__init__(self)
        RpmMechanismIO.__init__(self, name)

        # Sanity / defaults check
        constants = self._validate_constants(constants, name)

        # General attributes
        self.setName(name)
        self._long_name = long_name or name  # Typically for logging/smartdashboard such as "intake/indexer"
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._inputs = RpmMechanismIO.RpmMechanismIOInputs()

        self._constants = constants
        self._controller_type = controller_type

        self._physics_controller = None

        # Set up the motor controller
        match controller_type:
            case ControllerType.SparkMax:
                self._motor = SparkMax(self._device_id, SparkFlex.MotorType.kBrushless)
                if RobotBase.isSimulation():
                    self._sim_motor = SparkMaxSim(self._motor, motor)

            case ControllerType.SparkFlex:
                self._motor = SparkFlex(self._device_id, SparkFlex.MotorType.kBrushless)

                if RobotBase.isSimulation():
                    self._sim_motor = SparkFlexSim(self._motor, motor)
            case _:
                raise NotImplementedError(f"Unsupported motor type: {controller_type}")

        persist = PersistMode.kPersistParameters if persist_config else PersistMode.kNoPersistParameters

        config_status = try_until_ok(name, 5,
                                     lambda: self._motor.configure(self._motor_config(coast),
                                                                   ResetMode.kResetSafeParameters,
                                                                   persist))

        # Check if the device was successfully configured and can be reached over the
        # CAN bus.
        self._is_connected = self._check_is_connected(config_status)

        # Set up the encoders
        self._encoder: SparkRelativeEncoder = self._motor.getEncoder()

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._pid_controller: SparkClosedLoopController = self._motor.getClosedLoopController()
        self._pid_controller.setSetpoint(0.0, SparkBase.ControlType.kVoltage, ClosedLoopSlot(0))

        # The critical attributes/properties for operation
        self._velocity_goal: revolutions_per_minute = 0.0
        self._velocity_tolerance: revolutions_per_minute = 0.0

        # SysID Support
        self._sysid_routine = SysIdRoutine(SysIdRoutine.Config(),
                                           SysIdRoutine.Mechanism(lambda voltage: self._set_voltage(voltage),
                                                                  lambda log: self._log_motor(log),
                                                                  self,
                                                                  name))

    @staticmethod
    def _validate_constants(constants: Any, obj_name: str) -> Any:
        """
        Validate that the constants passed in have values/properties this class needs. They
        can be None if you want this class to use a default value (often zero), but they do need
        to exist as an explicit attribute of the object passed in
        """
        required = ["MAX_RPM", "LIMIT_CURRENT", "PROPORTIONAL_COEFFICIENT",
                    "INTEGRAL_COEFFICIENT", "DERIVATIVE_COEFFICIENT"]
        for attribute in required:
            # Needs to be there, even if set to None
            assert hasattr(constants, attribute), f"{attribute} was not found in {obj_name} object constants"

            # If set to None, use our default values (which may be None as well)
            if getattr(constants, attribute, None) is None:
                setattr(constants, attribute, getattr(DefaultConstants(), obj_name))

        return constants

    def _motor_config(self, coast: bool) -> SparkBaseConfig:
        """
        Motor config for the intake Indexer. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        match self._controller_type:
            case ControllerType.SparkMax:
                config = SparkMaxConfig()

            case ControllerType.SparkFlex:
                config = SparkFlexConfig()

        config = (config
                  .inverted(self._inverted)
                  .smartCurrentLimit(self._constants.LIMIT_CURRENT)
                  .setIdleMode(SparkFlexConfig.IdleMode.kCoast if coast else SparkFlexConfig.IdleMode.kBrake)
                  )

        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

        # Closed loop configuration parameters, slot=0
        #
        #   P:     If you’re not where you want to be, get there.
        #
        #   I:     If you haven’t been where you want to be for a while, apply more effort
        #          to get there”, since it really isn’t about speed.
        #
        #   D:     If you’re getting close to where you want to be, slow down.
        #
        #   IZone: If you are really far from where you want to be, don’t start applying
        #          more effort to get there until you are within this margin
        #
        slot0 = ClosedLoopSlot(ClosedLoopSlot.kSlot0)
        (
            config.closedLoop
            # .IMaxAccum(0.03, slot=slot0)
            # .IZone(3, slot=slot0)
            .pid(p=self._constants.PROPORTIONAL_COEFFICIENT,  # Slot 0 for position control
                 i=self._constants.INTEGRAL_COEFFICIENT,
                 d=self._constants.DERIVATIVE_COEFFICIENT,
                 slot=slot0)
            .outputRange(-1, 1)
        )
        # Apply any optional config
        if self._constants.VELOCITY_FEEDFORWARD is not None:
            config = config.closedLoop.velocityFF(self._constants.VELOCITY_FEEDFORWARD,
                                                  slot=slot0)

        if self._constants.IMAX_ACCUM is not None:
            config = config.IMaxAccum(self._constants.IMAX_ACCUM, slot=slot0)

        if self._constants.IZONE is not None:
            config = config.IZone(self._constants.IZONE, slot=slot0)

        # Set the encoder to return its position in radians
        config.encoder.positionConversionFactor(2 * math.pi)
        return config

    def _check_is_connected(self, config_status: REVLibError | None) -> bool:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        match self._controller_type:
            case ControllerType.SparkFlex | ControllerType.SparkMax:
                version = self._motor.getFirmwareVersion()
                logger.info(f"{self.getName()} firmware version: {version}")

                ok = (version != 0 and (config_status is None or
                                        config_status == REVLibError.kOk))
                # or RobotBase.isSimulation()
                if not ok:
                    logger.warning(f"{self.getName()} firmware version: {version}, status: {config_status}")
                return ok

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        match self._controller_type:
            case (ControllerType.SparkFlex, ControllerType.SparkMax):
                return self._is_connected
        return False

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
        rps = self._encoder.getVelocity()
        return -rps if self._inverted else rps

    @property
    def position(self) -> radians:
        return self._encoder.getPosition()

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
        self._velocity_tolerance = rpm_tolerance or 0.0
        self._velocity_goal, previous = max(0.0, min(self._constants.MAX_RPM, abs(rpm))), self._velocity_goal

        if self._velocity_goal != previous or self._velocity_tolerance != self.tolerance:
            logger.info(f"{self.getName()}: Setting goal RPM to {self._velocity_goal}. previous: {previous}")
            logger.info(
                f"{self.getName()}: current PID controller setpoint before command: {self._pid_controller.getSetpoint()}")

            self._pid_controller.setSetpoint(self._velocity_goal, SparkBase.ControlType.kVelocity)

    def stop(self) -> None:
        logger.info(f"{self.getName()}: Stop command was called")
        self._set_velocity_goal(0, 0)
        self._motor.disable()

    # TODO: Add support for getting any faults so we can display them back to the user and
    #       possibly clean them on startup if they are sticky

    def periodic(self) -> None:
        LogTracer.resetOuter(f"{self.getName()} periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs(self.getName(), self._inputs)
        LogTracer.record("UpdateInputs")

        # TODO: Look what we need to do if we provide replay?

        Logger.recordOutput(f"{self._long_name}/goal", self.goal)
        Logger.recordOutput(f"{self._long_name}/current", self.velocity_in_rpm)
        Logger.recordOutput(f"{self._long_name}/tolerance", self.tolerance)
        LogTracer.recordTotal()

        # Update SmartDashboard for this subsystem at a rate slower than the period
        counter = self._robot.counter
        if counter % 100 == 0 or (self._robot.counter % 7 == 0 and
                                  self._robot.isEnabled()):
            self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        self.dashboard_periodic()

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        SmartDashboard.putNumber(f"{self._long_name}/Goal", self.goal)
        SmartDashboard.putNumber(f"{self._long_name}/Current", self.velocity_in_rpm)
        SmartDashboard.putNumber(f"{self._long_name}/Tolerance", self.tolerance)
        SmartDashboard.putNumber(f"{self._long_name}/Voltage", self._motor.getAppliedOutput())
        SmartDashboard.putNumber(f"{self._long_name}/Current", self._motor.getOutputCurrent())

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        inputs.mechanism_connected = self.is_connected

        inputs.mechanism_position = self.position
        inputs.mechanism_velocity = self.velocity_in_rps
        inputs.mechanism_applied_voltage = self._motor.getAppliedOutput()
        inputs.mechanism_supply_current = self._motor.getOutputCurrent()

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
