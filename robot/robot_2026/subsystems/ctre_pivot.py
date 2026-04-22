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
from typing import Optional

from commands2 import cmd, Subsystem
from commands2.command import Command
from commands2.sysid import SysIdRoutine

from phoenix6 import SignalLogger, swerve, units, utils, StatusCode, StatusSignal
from phoenix6.hardware import TalonFX
from phoenix6.configs import TalonFXConfiguration, CurrentLimitsConfigs, MotorOutputConfigs
from phoenix6.signals import InvertedValue

from wpilib import RobotBase, RobotController, SmartDashboard
from wpilib.simulation import BatterySim, RoboRioSim, SingleJointedArmSim
from wpilib.sysid import State
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, degrees, degrees_per_second, degreesToRadians, inches, inchesToMeters, kilograms, \
    meters, radians, revolutions_per_minute, seconds, volts

from lib_6107.pykit.logger import Logger
from lib_6107.pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from lib_6107.subsystems.pykit.rotation_mechanism_io import RotationMechanismIO
from lib_6107.util.competition import event_active
from lib_6107.util.phoenix6_utils import try_until_ok, handle_faults
from lib_6107.util.phoenix6_signals import Phoenix6Signals

from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


class PivotConstants:
    TARGET_RPM: revolutions_per_minute = 60

    PROPORTIONAL_COEFFICIENT = .001     # kP - If you’re not where you want to be, get there.
    INTEGRAL_COEFFICIENT = 0    # 1e-5  # kI - If you haven’t been where you want to be for a while, apply more effort
                                        #      to get there”, since it really isn’t about speed.
    DERIVATIVE_COEFFICIENT = 1e-2       # kD - If you’re getting close to where you want to be, slow down.

    LIMIT_CURRENT: amperes = 30

    MAX_RPM: revolutions_per_minute = 6000
    GEAR_RATIO = 6.81
    DEPLOYED_ANGLE: degrees = 90.0  # This is straight forward since our encoder is set to zero on power up position
    RETRACTED_ANGLE: degrees = 0.0  # This is straight up.
    TOLERANCE: degrees = 2.0

    PIVOT_LEFT_ROOT_X: meters = inchesToMeters(.5)  # Bottom left corner of robot is (0, 0)
    PIVOT_RIGHT_ROOT_X: meters = inchesToMeters(24.5)
    PIVOT_ROOT_Y: meters = inchesToMeters(24.5)
    PIVOT_BASE_LENGTH: meters = inchesToMeters(8.0)  # TODO: Validate all this locations/lengths

    PIVOT_LENGTH: meters = inchesToMeters(8.0)
    PIVOT_MASS: kilograms = 1.0  # TODO: Verify

#@autologgable_output
class CtreIntakePivot(Subsystem, RotationMechanismIO):
    """
    Intake Pivot Motor.

    This subsystem has a CTRE Kraken-X60 motor and is using the internal encoder.
    Gear Ratio is 6.81. Encoder conversion factors are set up to count in degrees.
    So when you input a setpoint to the PID, that value is the angle (degrees).
    Positive moves the pivot forward/down. Setting back to zero returns it to the
    zero-point (see note below).

    The Idle mode is currently set to 'Coast'.

    IMPORTANT NOTE:  On motor power-on, the internal encoder will assume that the
                     current position is the 0 degrees. For the intake to run off
                     of commands that pass in an angle, always start with the intake
                     in the 'up' position. That is where it will need to start
                     anyway when starting a competition.

                     A 'chooser' on the operator console will have a default setting
                     that disables the XBox Controller buttons (POV up/down) when we
                     are not in competition. So that must be set to "True" to enable
                     the intake Pivot.

                     TODO: Need to work on logic so that when we are in competition,
                           the Intake pivot is automatically enabled (ready to go...).
    """

    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        self._initialized = False

        Subsystem.__init__(self)
        RotationMechanismIO.__init__(self, "IntakePivot")

        # General attributes
        self.setName("IntakePivot")
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._closed_loop = True        # Autonomous runs as a closed loop
        self._inputs = RotationMechanismIO.RotationMechanismIOInputs()

        self._position_goal = PivotConstants.RETRACTED_ANGLE

        self._physics_controller = None

        # Set up the motor controller
        self._motor = TalonFX(self._device_id, "rio")
        config: TalonFXConfiguration = self._motor_config(self._inverted)
        config_status = try_until_ok("Intake Pivot", 5,
                                     lambda: self._motor.configurator.apply(config))

        # Check if the device was successfully configured and can be reached over the
        # CAN bus.
        self._is_connected = self._check_is_connected(config_status)

        # Support simulation
        self._sim_initial = None

        # if RobotBase.isSimulation():              TODO: Future support
        #     gearbox = DCMotor.krakenX60(1)
        #     self._sim_motor = SparkFlexSim(self._motor, gearbox)
        #
        #     self._sim_encoder = SparkRelativeEncoderSim(self._motor)
        #
        #     moi = SingleJointedArmSim.estimateMOI(PivotConstants.PIVOT_LENGTH, PivotConstants.PIVOT_MASS)
        #     self._sim_pivot = SingleJointedArmSim(gearbox,
        #                                                 PivotConstants.GEAR_RATIO,
        #                                                 moi,
        #                                                 PivotConstants.PIVOT_LENGTH,
        #                                                 self._adjust_intake_angle_radians(
        #                                                     PivotConstants.DEPLOYED_ANGLE),
        #                                                 self._adjust_intake_angle_radians(
        #                                                     PivotConstants.RETRACTED_ANGLE),
        #                                                 True,
        #                                                 self._adjust_intake_angle_radians(
        #                                                     PivotConstants.RETRACTED_ANGLE))

        # The critical attributes/properties for operation
        self._applied_output: StatusSignal[volt] = self._motor.get_motor_voltage(False)
        self._velocity: StatusSignal[rotations_per_second] = self._motor.get_velocity(False)
        self._supply_current: StatusSignal[ampere] = self._motor.get_supply_current(False)
        self._position: StatusSignal[rotation] = self._motor.get_position(False)

        status = StatusSignal.set_update_frequency_for_all(1.0 / container.robot.period,
                                                           self._applied_output,
                                                           self._velocity,
                                                           self._supply_current,
                                                           self._position)
        if status != StatusCode.OK:
            status = self._motor.optimize_bus_utilization()

            if status != StatusCode.OK:
                logger.warning("%s: Error during signal bus optimization: %s", self.getName(), status)

        Phoenix6Signals.register_signals(self._applied_output,
                                         self._velocity,
                                         self._supply_current,
                                         self._position)
        #####################################
        # Visualization support       TODO: Future support
        #
        # mech_2d = LoggedMechanism2d(inchesToMeters(20), inchesToMeters(50))
        # mech_root = mech_2d.getRoot("Right Pivot Root",
        #                                   PivotConstants.PIVOT_ROOT_X,
        #                                   PivotConstants.PIVOT_ROOT_Y)
        # self._mech_base = LoggedMechanismLigament2d("Right Pivot Base",
        #                                                   PivotConstants.PIVOT_BASE_LENGTH,
        #                                                   angle,
        #                                                   color=Color8Bit(Color.kBlue))
        # mech_root.append(self._mech_base)
        #
        # SmartDashboard.putData("Right-Pivot", mech_2d)

        self._enable_chooser = LoggedNetworkBoolean("Intake/Enabled",
                                                    defaultValue=event_active())
        self._initialized = True

    @staticmethod
    def _get_rel_angle(target_abs: degrees, parent_abs: degrees) -> degrees:
        """
        Calculates the relative angle needed for a ligament given the
        desired absolute angle and the parent's absolute angle.
        """
        return target_abs - parent_abs

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self.is_initialized and self._enable_chooser.value

    @staticmethod
    def _motor_config(inverted: bool) -> TalonFXConfiguration:
        """
        Motor config for the intake pivot.
        """
        limit_configs = CurrentLimitsConfigs()
        limit_configs.stator_current_limit = PivotConstants.LIMIT_CURRENT
        limit_configs.stator_current_limit_enable = True

        motor_output: MotorOutputConfigs = MotorOutputConfigs()
        motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE if inverted else InvertedValue.COUNTER_CLOCKWISE_POSITIVE

        config = (TalonFXConfiguration()
                  .with_current_limits(limit_configs)
                  )
        config.motor_output = motor_output

        slot0 = config.slot0
        slot0.kP = PivotConstants.PROPORTIONAL_COEFFICIENT
        slot0.kI = PivotConstants.INTEGRAL_COEFFICIENT
        slot0.kD = PivotConstants.DERIVATIVE_COEFFICIENT
        slot0.kV = 0.12 # Velocity gain
        slot0.kA = 0.01 # Acceleration gain

        config.motion_magic.motion_magic_cruise_velocity = PivotConstants.TARGET_RPM/60  # In rotations/sec
        config.motion_magic.motion_magic_velocity = 3                                    # r/s^2    TODO: Experiment

        return config

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        return self._is_connected  # Need both to work

    def _check_is_connected(self, config_status: StatusCode | None) -> bool:
        return config_status == StatusCode.OK or (config_status is None and
                                                  StatusSignal.is_all_good(self._applied_output,
                                                                           self._velocity,
                                                                           self._supply_current,
                                                                           self._position))

    @property
    def angle(self) -> degrees:
        position: rotation = self._position.value       # TODO: Use 'inputs' value in future
        return position / 360

    @property
    def angular_velocity(self) -> degrees_per_second:
        velocity: rotations_per_second = self._position.value     # TODO: Use 'inputs' value in future
        return velocity / 360

    def _adjust_intake_angle(self, angle: degrees) -> degrees:
        """
        Since we start with the intake up and the encoder considers that 0 degrees
        and not 90, then we need to adjust
        """
        return 90.0 - angle

    def _adjust_intake_angle_radians(self, angle: degrees) -> radians:
        """
        Since we start with the intake up and the encoder considers that 0 degrees
        and not 90, then we need to adjust
        """
        return degreesToRadians(self._adjust_intake_angle(angle))

    def reset(self) -> None:
        self.stop()
        logger.info(f"Intake: Reset command was called  *** *** *** ***   NOT YET SUPPORTED")

        self._position_goal = PivotConstants.RETRACTED_ANGLE
        raise NotImplementedError("pivot reset: Reset command was called but is not supported at this time")

        #self._encoder.setPosition(0.0)
        # self._mech_base.setAngle(PivotConstants.RETRACTED_ANGLE)
        # self._pid_controller.reset()

    @property
    def position(self) -> degrees:
        return self._inputs.mechanism_position

    def pivot_up(self):
        # Set encoders to zero and go up ~90 degrees
        logger.info(f"Intake: Pivot up, currently at: {self.position}")
        logger.info(f"Intake: Pivot up. Position goal before command is: {self._position_goal}")

        self.set_position_goal(PivotConstants.RETRACTED_ANGLE)

    def pivot_tweak_up(self, increment: degrees = 1.0)-> None:
        self.set_position_goal(self._position_goal + increment)

    def pivot_tweak_down(self, increment: degrees = 1.0)-> None:
        self.set_position_goal(self._position_goal - increment)

    def pivot_down(self):
        # Set encoders to 90 degrees and go down
        logger.info(f"Intake: Pivot down, currently at: {self.position}")
        logger.info(f"Intake: Pivot down. Position goal before command is: {self._position_goal}")

        self.set_position_goal(PivotConstants.DEPLOYED_ANGLE)

    def set_position_goal(self, goal: degrees) -> None:
        if self._position_goal != goal:
            logger.info(f"Intake: Setting goal position to {goal}. currently at: {self.position}")
            self._position_goal = goal
            self.position = goal

    def at_deployed_angle(self) -> bool:
        pos = self._inputs.mechanism_position
        return pos <= PivotConstants.DEPLOYED_ANGLE + PivotConstants.TOLERANCE

    def at_retracted_angle(self) -> bool:
        pos = self._inputs.mechanism_position
        return pos >= PivotConstants.RETRACTED_ANGLE - PivotConstants.TOLERANCE

    def stop(self) -> None:
        logger.info(f"Intake Pivot: Stop command was called")
        self._motor.stopMotor()

    def periodic(self) -> None:
        if not self.is_initialized:
            return

        LogTracer.resetOuter("Intake Pivot periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Intake Pivot", self._inputs)
        LogTracer.record("UpdateInputs")

        # Update visualization
        # self._mech_base.setAngle(self._adjust_intake_angle(self._inputs.mechanism_2_position))

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Intake/Pivot/goal", self._position_goal)
        LogTracer.recordTotal()

        # Update SmartDashboard for this subsystem at a rate slower than the period
        counter = self._robot.counter
        if counter % 100 == 0 or (self._robot.counter % 29 == 0 and
                                  self._robot.isEnabled()):
            self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        SmartDashboard.putNumber("Intake/Pivot/goal",0.0)
        SmartDashboard.putNumber("Intake/Pivot/position", 0.0)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        SmartDashboard.putNumber("Intake/Pivot/goal", self._position_goal)
        SmartDashboard.putNumber("Intake/Pivot/position", self._position.value)
        # SmartDashboard.putNumber("Intake/Pivot/speed", self._inputs.mechanism_speed)

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        # TODO: Anything

    # def simulationPeriodic(self) -> None:         TODO: Future
    #     """
    #     This method is called periodically by the CommandScheduler (after the periodic
    #     function). It is useful for updating subsystem-specific state that needs to be
    #     maintained for simulations, such as for updating simulation classes and setting
    #     simulated sensor readings.
    #
    #     Unlike the physics 'update_sim', it is not called with the current time (now)
    #     or the amount of time since 'update_sim' was called (tm_diff).  It is called
    #     just after the 'periodic' call and before the 'update_sim' is called. One other
    #     'important' difference is 'update_sim' is called at a period >= 10 ms instead
    #     of the default 20 mS for the CommandScheduler's simulationPeriodic (this function).
    #     """
    #     if self._initialized and self._robot.isEnabled() and self._sim_pivot is not None and self._sim_pivot is not None:
    #         LogTracer.resetOuter(f"{self.getName()}-simulationPeriodic")
    #
    #         output = self._sim_motor.getAppliedOutput()
    #
    #         if self._sim_initial is None:
    #             self._encoder.setPosition(PivotConstants.RETRACTED_ANGLE)
    #             self._sim_initial = self._encoder.getPosition()
    #
    #         input_voltage = RobotController.getInputVoltage()  # TODO: Can we use BatterySim?
    #         applied_output = output * input_voltage
    #
    #         if applied_output != 0.0:
    #             self._sim_motor.iterate(applied_output, input_voltage, self._period)
    #
    #         self._sim_pivot.setInputVoltage(applied_output)
    #         self._sim_pivot.update(self._period)
    #
    #         # Set the simulated encoder
    #         # self._sim_encoder.setPosition(self._sim_pivot.getAngleDegrees())
    #         self._encoder.setPosition(self._sim_pivot.getAngleDegrees())
    #
    #         pivot = self._sim_pivot.getAngleDegrees()
    #
    #         # And simulate current drain
    #         RoboRioSim.setVInVoltage(BatterySim.calculate([self._sim_pivot.getCurrentDraw()]))
    #         LogTracer.recordTotal()

    def updateInputs(self, inputs: RotationMechanismIO.RotationMechanismIOInputs) -> None:
        if not self.is_initialized:
            return
        inputs.mechanism_connected = StatusSignal.is_all_good(self._applied_output,
                                                              self._velocity,
                                                              self._supply_current,
                                                              self._position)
        inputs.mechanism_position = self._position.value
        inputs.mechanism_speed = self._velocity.value
        inputs.mechanism_applied_voltage = self._applied_output.value
        inputs.mechanism_supply_current = self._supply_current.value
        # inputs.mechanism_torque_amps: amperes = 0.0
        # TODO: Figure this out or drop it inputs.mechanism_torque_amps = self._motor.get

    # def update_sim(self, now: float, tm_diff: float) -> None:
    #     """
    #     Called when the simulation parameters for the program need to be updated.
    #     This function is called from the '_simulationPeriodic' function of the
    #     robotpy core routine and is called at a period >= 10 mS. Note that the
    #     CommandScheduler also has an 'simulationPeriodic' function that it calls
    #     into all Command2 based subsystems at its update period which has a
    #     default rate of 20 mS.
    #
    #     This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
    #     that function uses pykit's logging method, you should use those values in
    #     your simulation.
    #
    #     :param now:     The current time as a float
    #     :param tm_diff: The amount of time that has passed since the last
    #                     time that this function was called
    #     """
    #     pass

    # def set_position(self, position: inches) -> None:
    #     """
    #     Set the desired encoder position. This is primarily for Autonomous mode when
    #     we are running with a closed loop system
    #
    #     Args:
    #         position (rotations +/-): The desired number of rotations
    #     """
    #     # Limit to max/min
    #     position = max(min(position, PivotConstants.RETRACTED_ANGLE),
    #                    PivotConstants.DEPLOYED_ANGLE)
    #
    #     if position != self.position:
    #         logger.info(f"Intake: Setting position to {position}")
    #
    #         self._inputs.mechanism_position = position
    #         self._encoder.setPosition(position)
    #
    #         if RobotBase.isSimulation():
    #             self._sim_encoder.setPosition(position)
    #
    # def set_voltage(self, voltage: volts) -> None:
    #     """
    #     Set the drive voltage
    #     """
    #     if  voltage != self._motor.getAppliedOutput():
    #         logger.info(f"Intake: Setting voltage to {voltage}")
    #         self._motor.setVoltage(voltage)
    #
    # def sys_id_routine(self, subsystem: Subsystem) -> Command:
    #     """
    #     Model the behavior of the intake (for better control) by sweeping through
    #     the max and min heights.
    #     """
    #
    #     # TODO: Add to non-competition AUTO chooser
    #     def log_state(sys_id_state: State) -> None:
    #         match sys_id_state:
    #             case State.kQuasistaticForward:
    #                 state = "quasistatic-forward"
    #
    #             case State.kQuasistaticReverse:
    #                 state = "quasistatic-reverse"
    #
    #             case State.kDynamicForward:
    #                 state = "dynamic-forward"
    #
    #             case State.kDynamicReverse:
    #                 state = "dynamic-reverse"
    #
    #             case State.kNone:
    #                 state = "none"
    #
    #         Logger.recordOutput(f"Intake/SysID State", state)
    #
    #     characterization_routine = SysIdRoutine(SysIdRoutine.Config(0.5, 6, 10, log_state),
    #                                             SysIdRoutine.Mechanism(self.set_voltage,
    #                                                                    (lambda _: None),
    #                                                                    subsystem,
    #                                                                    "Intake Pivot"))
    #     return cmd.sequence(
    #         cmd.runOnce(lambda: self.set_closed_loop(False), self),
    #         characterization_routine.quasistatic(SysIdRoutine.Direction.kForward).until(
    #             lambda: self.at_retracted_angle()),
    #         characterization_routine.quasistatic(SysIdRoutine.Direction.kReverse).until(
    #             lambda: self.at_deployed_angle()),
    #         characterization_routine.dynamic(SysIdRoutine.Direction.kForward).until(
    #             lambda: self.at_retracted_angle()),
    #         characterization_routine.dynamic(SysIdRoutine.Direction.kReverse).until(
    #             lambda: self.at_deployed_angle()),
    #         cmd.runOnce(lambda: self.set_closed_loop(True), self),
    #     )

    def fault_detection(self, state: str, clear: Optional[bool] = True, notify: Optional[bool] = True) -> None:
        """
        This routine is responsible for reading any existing faults and based
        input parameters, report them for display, and possibly clear them

        All faults detected always results in a warning log message, so please be
        aware of this if you do not clear them

        TODO: Good thing for a base class, don't you think
        """
        # TODO
        # TODO: handle_faults("Pivot-Right Motor", state, self._motor, clear=clear, notify=notify)
