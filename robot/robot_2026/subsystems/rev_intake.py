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

from commands2 import Subsystem, cmd
from commands2.button import Trigger
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import PersistMode, ResetMode, SparkBase, SparkFlex, SparkFlexConfig, SparkRelativeEncoder
from wpilib import SendableChooser, SmartDashboard
from wpilib.sysid import State
from wpimath.controller import PIDController
from wpimath.units import amperes, revolutions_per_minute, volts, degrees, meters, inches, \
    inchesToMeters, degrees_per_second

from lib_6107.subsystems.pykit.rotation_mechanism_io import RotationMechanismIO
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)

class IntakeConstants:
    TARGET_RPM: revolutions_per_minute = 10
    PROPORTIONAL_COEFFICIENT = 0.1     # kP
    INTEGRAL_COEFFICIENT = 0.1         # kI
    DERIVATIVE_COEFFICIENT = 0.0       # kD
    LIMIT_CURRENT: amperes = 35
    IZONE_RANGE = 0.0
    GEAR_RATIO = 25.0
    DRIVE_VOLTAGE: volts = 0.10     # Start at 10% power
    SPOOL_DIAMETER: meters = 1.0    # TODO: Use an algorythm to compensate for cord already spooled in
    MIN_HEIGHT: degrees = 0.0
    MAX_HEIGHT: degrees = 90.0
    TOLERANCE: degrees = 2.0

@autologgable_output
class RevIntake(Subsystem, RotationMechanismIO):

    def __init__(self, container: 'RobotContainer',
                 can_left_device_id: int, can_right_device_id: int,
                 left_inverted: bool, right_inverted: bool) -> None:
        Subsystem.__init__(self)
        RotationMechanismIO.__init__(self, "Intake")

        # General attributes
        self.setName(self.__class__.__name__)
        self._container = container
        self._robot = container.robot
        self._left_device_id = can_left_device_id
        self._left_inverted = left_inverted
        self._right_device_id = can_right_device_id
        self._right_inverted = right_inverted
        self._closed_loop = True        # Autonomous runs as a closed loop
        self._left_inputs = RotationMechanismIO.RotationMechanismIOInputs()
        self._right_inputs = RotationMechanismIO.RotationMechanismIOInputs()

        # Set up the motor controller
        self._left_motor = SparkFlex(self._left_device_id, SparkBase.MotorType.kBrushless)
        self._left_motor.configure(self._motor_config(self._left_inverted),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kPersistParameters)

        self._right_motor = SparkFlex(self._right_device_id, SparkBase.MotorType.kBrushless)
        self._right_motor.configure(self._motor_config(self._right_inverted),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kPersistParameters)
        # Set up the encoders
        self._left_encoder: SparkRelativeEncoder = self._left_motor.getEncoder()
        self._left_encoder.setPosition(0.0)

        self._right_encoder: SparkRelativeEncoder = self._right_motor.getEncoder()
        self._right_encoder.setPosition(0.0)

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._left_pid_controller: PIDController = PIDController(IntakeConstants.PROPORTIONAL_COEFFICIENT,
                                                            IntakeConstants.INTEGRAL_COEFFICIENT,
                                                            IntakeConstants.DERIVATIVE_COEFFICIENT)
        self._left_pid_controller.setIZone(IntakeConstants.IZONE_RANGE)
        self._right_pid_controller: PIDController = PIDController(IntakeConstants.PROPORTIONAL_COEFFICIENT,
                                                            IntakeConstants.INTEGRAL_COEFFICIENT,
                                                            IntakeConstants.DERIVATIVE_COEFFICIENT)
        self._right_pid_controller.setIZone(IntakeConstants.IZONE_RANGE)

        # The critical attributes/properties for operation
        self._position_goal: degrees = 0.0
        self._applied_voltage: volts  = 0.0

        # TODO: Remove following once all works
        self._enable_intake = SendableChooser()
        self._enable_intake.setDefaultOption("False", False)
        self._enable_intake.addOption("True", True)
        SmartDashboard.putData("Intake Enabled", self._enable_intake)

    @property
    def enabled(self) -> bool:
        return self._enable_intake.getSelected()

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkFlexConfig:
        config = SparkFlexConfig()
        config.inverted(inverted)
        config.setIdleMode(SparkFlexConfig.IdleMode.kBrake)      # Set idle mode as brake
        config.limitSwitch.forwardLimitSwitchEnabled(False)
        config.limitSwitch.reverseLimitSwitchEnabled(False)
        config.smartCurrentLimit(IntakeConstants.LIMIT_CURRENT)

        # Set the position conversion factor (e.g., to convert rotations to degrees)
        # The native unit is rotations. So use 360.0 as conversion factor to get degrees
        config.encoder.positionConversionFactor(360.0)

        # Set the velocity conversion factor (e.g., to convert RPM to degrees/second)
        # The native unit is RPM. So use:  (360 degrees/revolution) / (60 seconds/minute) = 6
        config.encoder.velocityConversionFactor(6.0)
        return config

    def reset(self) -> None:
        self.stop()

        self._position_goal = 0
        self._left_encoder.setPosition(0.0)
        self._right_encoder.setPosition(0.0)
        self._left_pid_controller.reset()
        self._right_pid_controller.reset()

    @property
    def closed_loop(self) -> bool:
        return self._closed_loop

    def set_closed_loop(self, closed_loop: bool) -> None:
        self._closed_loop = closed_loop

    @property
    def left_position(self) -> degrees:
        return self._left_inputs.mechanism_position

    @property
    def right_position(self) -> degrees:
        return self._right_inputs.mechanism_position

    def set_position(self, position: inches) -> None:
        if self._position_goal != position:
            self._position_goal = position
            self._left_pid_controller.setSetpoint(position)
            self._right_pid_controller.setSetpoint(position)

    def at_min(self, left: bool) -> bool:
        position = self._left_inputs.mechanism_position if left else self._right_inputs.mechanism_position
        return position <= IntakeConstants.MIN_HEIGHT + \
            IntakeConstants.TOLERANCE

    def at_max(self, left: bool) -> bool:
        position = self._left_inputs.mechanism_position if left else self._right_inputs.mechanism_position
        return position >= IntakeConstants.MAX_HEIGHT - \
            IntakeConstants.TOLERANCE

    def stop(self) -> None:
        self._left_motor.stopMotor()
        self._right_motor.stopMotor()

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem periodic")

        self.updateLeftInputs(self._left_inputs)
        self.updateRightInputs(self._right_inputs)

        Logger.processInputs("Intake", self._inputs)
        LogTracer.record("UpdateInputs")

        if self._closed_loop:
            self.set_position(self._position_goal)

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Intake/goal", self._position_goal)
        Logger.recordOutput("Intake/ClosedLoop", self._closed_loop)
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
        SmartDashboard.putNumber("Intake/goal", self._position_goal)
        SmartDashboard.putNumber("Intake/left-position", self.left_position)
        SmartDashboard.putNumber("Intake/left-speed", self._left_inputs.mechanism_speed)
        SmartDashboard.putNumber("Intake/right-position", self.right_position)
        SmartDashboard.putNumber("Intake/right-speed", self._right_inputs.mechanism_speed)
        SmartDashboard.putBoolean("Intake/closed-loop", self._closed_loop)

    def updateLeftInputs(self, inputs: RotationMechanismIO.RotationMechanismIOInputs) -> None:
        inputs.mechanism_connected = True   # TODO: Figure this one out
        inputs.mechanism_position = self._left_encoder.getPosition()
        inputs.mechanism_speed = self._left_encoder.getVelocity()

        inputs.mechanism_applied_voltage = self._left_motor.getBusVoltage()
        inputs.mechanism_supply_current = self._left_motor.getOutputCurrent()
        # TODO: Figure this out or drop it inputs.mechanism_torque_amps = self._motor.get

    def updateRightInputs(self, inputs: RotationMechanismIO.RotationMechanismIOInputs) -> None:
        inputs.mechanism_connected = True   # TODO: Figure this one out
        inputs.mechanism_position = self._right_encoder.getPosition()
        inputs.mechanism_speed = self._right_encoder.getVelocity()

        inputs.mechanism_applied_voltage = self._right_motor.getBusVoltage()
        inputs.mechanism_supply_current = self._right_motor.getOutputCurrent()
        # TODO: Figure this out or drop it inputs.mechanism_torque_amps = self._motor.get

    def set_position(self, position: inches) -> None:
        """
        Set the desired encoder position. This is primarily for Autonomous mode when
        we are running with a closed loop system

        Args:
            position (rotations +/-): The desired number of rotations
        """
        # Limit to max/min
        position = max(min(position, IntakeConstants.MAX_HEIGHT),
                       IntakeConstants.MIN_HEIGHT)
        self._left_encoder.setPosition(position)
        self._right_encoder.setPosition(position)

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        self._left_motor.setVoltage(voltage)
        self._right_motor.setVoltage(voltage)

    def sysIdRoutine(self, subsystem: Subsystem, left: bool) -> Command:
        """Model the behavior of the climber (for better control) by sweeping through the max and min heights."""

        def log_state(state: State) -> None:
            state = ""
            match state:
                case State.kQuasistaticForward:
                    state = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    state = "quasistatic-reverse"
                case State.kDynamicForward:
                    state = "dynamic-forward"
                case State.kDynamicReverse:
                    state = "dynamic-reverse"
                case State.kNone:
                    state = "none"
            which = "Left" if left else "Right"
            Logger.recordOutput(f"Intake/{which}-Motor/SysID State", state)

        characterization_routine = SysIdRoutine(SysIdRoutine.Config(0.5, 6, 10, log_state),
                                                SysIdRoutine.Mechanism(self.set_voltage,
                                                                       (lambda _: None),
                                                                       subsystem))
        return cmd.sequence(
            cmd.runOnce(lambda: self.set_closed_loop(False), self),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kForward).until(lambda: self.at_max(left)),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kReverse).until(lambda: self.at_min(left)),
            characterization_routine.dynamic(SysIdRoutine.Direction.kForward).until(lambda: self.at_max(left)),
            characterization_routine.dynamic(SysIdRoutine.Direction.kReverse).until(lambda: self.at_min(left)),
            cmd.runOnce(lambda: self.set_closed_loop(True), self),
        )
