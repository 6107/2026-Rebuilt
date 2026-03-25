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

from commands2 import PrintCommand, Subsystem
from commands2.button import Trigger
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import ClosedLoopSlot, PersistMode, ResetMode, SparkBase, SparkClosedLoopController, SparkFlex, \
    SparkFlexConfig, SparkFlexSim, SparkRelativeEncoder
from wpilib import RobotBase, SmartDashboard
from wpilib.simulation import BatterySim, RoboRioSim
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, radiansPerSecondToRotationsPerMinute, revolutions_per_minute, seconds, volts

from lib_6107.subsystems.pykit.rpm_mechanism_io import RpmMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


class IntakeConstants:
    PROPORTIONAL_COEFFICIENT = 10  # kP
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0  # kD
    LIMIT_CURRENT: amperes = 30

    GEAR_REDUCTION = 6.75  # TODO: Get number
    MEASUREMENT_STD_DEV = [0.0, 0.0]  # TODO: Get number for noise
    MAX_RPM = 6784
    TARGET_RPM: revolutions_per_minute = 100  # Start slow


@autologgable_output
class RevIntakeRoller(Subsystem, RpmMechanismIO):
    """
    Intake Roller Motor.

    This is the rolly-grabber at the front of the intake.
    """

    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        Subsystem.__init__(self)
        RpmMechanismIO.__init__(self, "IntakeRoller")

        # General attributes
        self.setName(self.__class__.__name__)
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._closed_loop = True  # Autonomous runs as a closed loop (only false
        self._inputs = RpmMechanismIO.RpmMechanismIOInputs()

        self._physics_controller = None

        # Set up the motor controller
        self._motor = SparkFlex(self._device_id, SparkFlex.MotorType.kBrushless)
        try_until_ok("Intake Roller", 5,
                     lambda: self._motor.configure(self._motor_config(self._inverted),
                                                   ResetMode.kResetSafeParameters,
                                                   PersistMode.kNoPersistParameters))
        # Set up the encoders
        self._encoder: SparkRelativeEncoder = self._motor.getEncoder()

        # Support simulation
        if RobotBase.isSimulation():
            self._sim_motor = SparkFlexSim(self._motor, DCMotor.NEO(1))

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._pid_controller: SparkClosedLoopController = self._motor.getClosedLoopController()
        self._pid_controller.setSetpoint(0, SparkBase.ControlType.kVoltage, ClosedLoopSlot(0))

        # The critical attributes/properties for operation
        self._rpm_goal: revolutions_per_minute = 0.0
        self._applied_voltage: volts = 0.0

        # # TODO:  Next is not currently supported
        self._sysid_routine = None
        # self._sysid_routine = SysIdRoutine(SysIdRoutine.Config(),
        #                                    SysIdRoutine.Mechanism(self.log_motor,
        #                   lambda voltage: self._motor.setVoltage(voltage),
        #                                                           self))
        #####################################
        # Visualization support  TODO: ??

    def log_motor(self, log):
        log.motor("roller").field("velocity", self._encoder.getVelocity())

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self._container.intake_pivot.enabled  # Use a single enable for all the intake frontend

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkFlexConfig:
        """
        Motor config for the intake roller. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        config = (SparkFlexConfig().
                  inverted(inverted).
                  smartCurrentLimit(IntakeConstants.LIMIT_CURRENT).
                  setIdleMode(SparkFlexConfig.IdleMode.kCoast)
                  )
        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

        # Closed loop configuration parameters, slot=0
        # TODO: Use SysID to determine actual values
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
            config.closedLoop.
            # IMaxAccum(0.03, slot=slot0).
            # IZone(3, slot=slot0).
            pid(p=IntakeConstants.PROPORTIONAL_COEFFICIENT,  # Slot 0 for position control
                i=IntakeConstants.INTEGRAL_COEFFICIENT,
                d=IntakeConstants.DERIVATIVE_COEFFICIENT,
                slot=slot0)
            .outputRange(-1, 1)
        )
        return config

    def reset(self) -> None:
        self.stop()
        logger.info(f"Intake Roller: Reset command was called  *** *** *** ***   NOT YET SUPPORTED")
        self._rpm_goal = 0

    @property
    def closed_loop(self) -> bool:
        return self._closed_loop

    def set_closed_loop(self, closed_loop: bool) -> None:
        self._closed_loop = closed_loop

    def intake_fuel(self):
        # Consume fuel from the playing area
        logger.info(f"Intake Roller: Intake fuel, currently at: {self._encoder.getPosition()}")
        logger.info(f"Intake Roller: Intake fuel. RPM goal before command is: {self._rpm_goal}")

        self.set_rpm_goal(self._rpm_goal)

    def expel_fuel(self):
        # Expel any fuel we may have
        logger.info(f"Intake Roller: Expel fuel, currently at: {self._encoder.getPosition()}")
        logger.info(f"Intake Roller: Expel fuel. RPM goal before command is: {self._rpm_goal}")

        self.set_rpm_goal(-self._rpm_goal)  # TODO: Should this be separate or maybe lower

    def set_rpm_goal(self, goal: revolutions_per_minute) -> None:
        if self._rpm_goal != goal:
            logger.info(f"Intake Roller: Setting goal RPM to {goal}. currently at: {self._encoder.getPosition()}")
            logger.info(
                f"Intake Roller: current PID controller setpoint before command is: {self._pid_controller.getSetpoint()}")

            self._rpm_goal = goal

            self._pid_controller.setSetpoint(goal, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
            # TODO: What about velocity goal?
            #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning

    def stop(self) -> None:
        logger.info(f"Intake Roller: Stop command was called")
        self._motor.stopMotor()

    def periodic(self) -> None:
        LogTracer.resetOuter("Intake Roller periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Intake Roller", self._inputs)
        LogTracer.record("UpdateInputs")

        # Update visualization
        pass

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Intake/Roller/goal", self._rpm_goal)
        Logger.recordOutput("Intake/Roller/closed-loop", self._closed_loop)
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
        pass

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        SmartDashboard.putNumber("Intake/Roller/goal", self._rpm_goal)
        SmartDashboard.putBoolean("Intake/Roller/closed-loop", self._closed_loop)
        SmartDashboard.putNumber("Intake/Roller/RPM", self._inputs.drive_velocity)

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        # TODO: Anything

    def simulationPeriodic(self) -> None:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function). It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called. One other
        'important' difference is 'update_sim' is called at a period >= 10 ms instead
        of the default 20 mS for the CommandScheduler's simulationPeriodic (this function).
        """
        pass

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        inputs.drive_connected = True  # TODO: Figure this one out
        inputs.encoder_connected = True  # TODO: Figure this one out
        inputs.drive_velocity = self._encoder.getVelocity()
        inputs.drive_applied = self._motor.getBusVoltage()
        inputs.drive_supply_current = self._motor.getOutputCurrent()

    def update_sim(self, now: float, tm_diff: float) -> None:
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
            pass
            voltage = RoboRioSim.getVInVoltage()
            rpm = radiansPerSecondToRotationsPerMinute(self._sim_motor.getVelocity())
            self._sim_motor.iterate(rpm, voltage, tm_diff)

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._sim_motor.getMotorCurrent()]))

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        if voltage != self._motor.getAppliedOutput():
            logger.info(f"Intake Roller: Setting voltage to {voltage}")
            self._motor.setVoltage(voltage)

    def sys_id_routine(self) -> Command:
        """
        Model the behavior of the intake (for better control) by sweeping through
        the max and min heights.
        """
        if self._sysid_routine is not None:
            return self._sysid_routine.quasistatic(SysIdRoutine.Direction.kForward)

        logger.error(f"Intake Roller: SysID routine is not currently supported")
        return PrintCommand("Intake Roller: SysID routine is not currently supported")
