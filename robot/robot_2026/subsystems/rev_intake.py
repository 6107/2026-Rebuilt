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

from commands2 import cmd, Subsystem
from commands2.button import Trigger
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from rev import ClosedLoopSlot, PersistMode, ResetMode, SparkBase, SparkClosedLoopController, SparkFlex, \
    SparkFlexConfig, SparkFlexSim, SparkRelativeEncoder, SparkRelativeEncoderSim
from wpilib import Color, Color8Bit, RobotBase, SmartDashboard, SendableChooser
from wpilib.simulation import BatterySim, RoboRioSim, SingleJointedArmSim
from wpilib.sysid import State
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, degrees, inches, inchesToMeters, kilograms, meters, revolutions_per_minute, seconds, \
    volts

from lib_6107.pykit.LoggedMechanism2d import LoggedMechanism2d
from lib_6107.pykit.LoggedMechanismLigament2d import LoggedMechanismLigament2d
from lib_6107.subsystems.pykit.dual_mechanism_io import DualMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)

class IntakeConstants:
    TARGET_RPM: revolutions_per_minute = 10
    PROPORTIONAL_COEFFICIENT = 0.1     # kP
    INTEGRAL_COEFFICIENT = 0.1         # kI
    DERIVATIVE_COEFFICIENT = 0.0       # kD
    LIMIT_CURRENT: amperes = 30
    IZONE_RANGE = 0.0
    GEAR_RATIO = 25.0
    DRIVE_VOLTAGE: volts = 0.10     # Start at 10% power
    SPOOL_DIAMETER: meters = 1.0    # TODO: Use an algorythm to compensate for cord already spooled in
    DEPLOYED_ANGLE: degrees = 0.0
    RETRACTED_ANGLE: degrees = 90.0
    TOLERANCE: degrees = 2.0

    PIVOT_GEARING = 2.0  # Verify
    PIVOT_LEFT_ROOT_X: meters = inchesToMeters(.5)  # Bottom left corner of robot is (0, 0)
    PIVOT_RIGHT_ROOT_X: meters = inchesToMeters(24.5)
    PIVOT_ROOT_Y: meters = inchesToMeters(24.5)
    PIVOT_BASE_LENGTH: meters = inchesToMeters(8.0)  # TODO: Validate all this locations/lengths

    PIVOT_LENGTH: meters = inchesToMeters(8.0)
    PIVOT_MASS: kilograms = 1.0  # TODO: Verify

@autologgable_output
class RevIntake(Subsystem, DualMechanismIO):

    def __init__(self, container: 'RobotContainer',
                 can_left_device_id: int, can_right_device_id: int,
                 left_inverted: bool, right_inverted: bool) -> None:
        Subsystem.__init__(self)
        DualMechanismIO.__init__(self, "Intake")

        # General attributes
        self.setName(self.__class__.__name__)
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._left_device_id = can_left_device_id
        self._left_inverted = left_inverted
        self._right_device_id = can_right_device_id
        self._right_inverted = right_inverted
        self._closed_loop = True        # Autonomous runs as a closed loop
        self._inputs = DualMechanismIO.DualMechanismIOInputs()

        self._physics_controller = None

        # Set up the motor controller
        self._left_motor = SparkFlex(self._left_device_id, SparkBase.MotorType.kBrushless)
        try_until_ok("Left Intake", 5,
                     lambda: self._left_motor.configure(self._motor_config(self._left_inverted),
                                                        ResetMode.kResetSafeParameters,
                                                        PersistMode.kNoPersistParameters))

        self._right_motor = SparkFlex(self._right_device_id, SparkBase.MotorType.kBrushless)
        try_until_ok("Right Intake", 5,
                     lambda: self._right_motor.configure(self._motor_config(self._right_inverted),
                                                         ResetMode.kResetSafeParameters,
                                                         PersistMode.kNoPersistParameters))
        # Set up the encoders
        self._left_encoder: SparkRelativeEncoder = self._left_motor.getEncoder()
        self._left_encoder.setPosition(0.0)

        self._right_encoder: SparkRelativeEncoder = self._right_motor.getEncoder()
        self._right_encoder.setPosition(0.0)


        # Support simulation
        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO(1)
            self._left_sim_motor = SparkFlexSim(self._left_motor, gearbox)
            self._right_sim_motor = SparkFlexSim(self._right_motor, gearbox)

            self._left_sim_encoder = SparkRelativeEncoderSim(self._left_motor)
            self._right_sim_encoder = SparkRelativeEncoderSim(self._right_motor)

            moi = SingleJointedArmSim.estimateMOI(IntakeConstants.PIVOT_LENGTH, IntakeConstants.PIVOT_MASS)
            self._left_sim_pivot = SingleJointedArmSim(gearbox,
                                                       IntakeConstants.PIVOT_GEARING,
                                                       moi,
                                                       IntakeConstants.PIVOT_LENGTH,
                                                       IntakeConstants.DEPLOYED_ANGLE,
                                                       IntakeConstants.RETRACTED_ANGLE,
                                                       True,
                                                       IntakeConstants.RETRACTED_ANGLE)
            self._right_sim_pivot = SingleJointedArmSim(gearbox,
                                                        IntakeConstants.PIVOT_GEARING,
                                                        moi,
                                                        IntakeConstants.PIVOT_LENGTH,
                                                        IntakeConstants.DEPLOYED_ANGLE,
                                                        IntakeConstants.RETRACTED_ANGLE,
                                                        True,
                                                        IntakeConstants.RETRACTED_ANGLE)

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._left_pid_controller: SparkClosedLoopController = self._left_motor.getClosedLoopController()
        self._right_pid_controller: SparkClosedLoopController = self._right_motor.getClosedLoopController()

        self._left_pid_controller.setReference(0, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
        self._right_pid_controller.setReference(0, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
        # TODO: What about velocity goal?


        # The critical attributes/properties for operation
        self._position_goal: degrees = 0.0
        self._applied_voltage: volts  = 0.0

        #####################################
        # Visualization support
        left_mech_2d = LoggedMechanism2d(20, 50)
        mech_root = left_mech_2d.getRoot("Left Pivot Root",
                                         IntakeConstants.PIVOT_LEFT_ROOT_X,
                                         IntakeConstants.PIVOT_ROOT_Y)
        self._left_mech_base = LoggedMechanismLigament2d("Left Pivot Base",
                                                         IntakeConstants.PIVOT_BASE_LENGTH,
                                                         90,
                                                         color=Color8Bit(Color.kBlue))
        mech_root.append(self._left_mech_base)

        right_mech_2d = LoggedMechanism2d(20, 50)
        mech_root = right_mech_2d.getRoot("Right Pivot Root",
                                                           IntakeConstants.PIVOT_RIGHT_ROOT_X,
                                                           IntakeConstants.PIVOT_ROOT_Y)
        self._right_mech_base = LoggedMechanismLigament2d("Right Pivot Base",
                                                          IntakeConstants.PIVOT_BASE_LENGTH,
                                                          90,
                                                          color=Color8Bit(Color.kBlue))
        mech_root.append(self._right_mech_base)

        SmartDashboard.putData("Left-Pivot", left_mech_2d)
        SmartDashboard.putData("Right-Pivot", right_mech_2d)

        # TODO: Remove following once all works
        # self._enable_intake = LoggedDashboardChooser("Intake Enable")
        self._enable_intake = SendableChooser()
        self._enable_intake.setDefaultOption("False", False)
        self._enable_intake.addOption("True", True)

        if isinstance(self._enable_intake, SendableChooser):
            SmartDashboard.putData("Intake Enabled", self._enable_intake)
        elif isinstance(self._enable_intake, LoggedDashboardChooser):
            pass

    @property
    def enabled(self) -> bool:
        return self._enable_intake.getSelected()

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkFlexConfig:
        config = (SparkFlexConfig().
                  inverted(inverted).
                  smartCurrentLimit(IntakeConstants.LIMIT_CURRENT).
                  setIdleMode(SparkFlexConfig.IdleMode.kBrake).
                  smartCurrentLimit(IntakeConstants.LIMIT_CURRENT)
                  )
        config.limitSwitch.forwardLimitSwitchEnabled(True).reverseLimitSwitchEnabled(True)

        # Set the position conversion factor (e.g., to convert rotations to degrees)
        # The native unit is rotations. So use 360.0 as conversion factor to get degrees
        config.encoder.positionConversionFactor(360.0)

        # Set the velocity conversion factor (e.g., to convert RPM to degrees/second)
        # The native unit is RPM. So use:  (360 degrees/revolution) / (60 seconds/minute) = 6
        config.encoder.velocityConversionFactor(6.0)

        # Closed loop configuration parameters, slot=0
        # TODO: Use SysID to determine actual values
        (
            config.closedLoop.
            pid(IntakeConstants.PROPORTIONAL_COEFFICIENT,  # Slot 0 for position control
                IntakeConstants.INTEGRAL_COEFFICIENT,
                IntakeConstants.DERIVATIVE_COEFFICIENT,
                ClosedLoopSlot(0))
            .outputRange(-0.2, 0.2)
        )

        # TODO: add to above -> feedForward.kS(0).kV(0).kA(0).kCos(0).kCosRatio(0))
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode

        return config

    def reset(self) -> None:
        self.stop()

        self._position_goal = 0
        self._left_encoder.setPosition(0.0)
        self._right_encoder.setPosition(0.0)
        self._left_mech_base.setAngle(90)
        self._right_mech_base.setAngle(90)
        # self._left_pid_controller.reset()
        # self._right_pid_controller.reset()

    @property
    def closed_loop(self) -> bool:
        return self._closed_loop

    def set_closed_loop(self, closed_loop: bool) -> None:
        self._closed_loop = closed_loop

    @property
    def left_position(self) -> degrees:
        return self._inputs.mechanism_1_position

    @property
    def right_position(self) -> degrees:
        return self._inputs.mechanism_2_position

    def pivot_up(self):
        # Set encoders to zero and go up ~90 degrees
        logger.info(f"Intake: Pivot up, currently at l/r: {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")
        self._left_encoder.setPosition(IntakeConstants.DEPLOYED_ANGLE)
        self._right_encoder.setPosition(IntakeConstants.DEPLOYED_ANGLE)
        self._position_goal = IntakeConstants.DEPLOYED_ANGLE
        self.set_position_goal(IntakeConstants.RETRACTED_ANGLE)

    def pivot_down(self):
        # Set encoders to 90 degrees and go down
        logger.info(f"Intake: Pivot down, currently at l/r: {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")
        self._left_encoder.setPosition(IntakeConstants.RETRACTED_ANGLE)
        self._right_encoder.setPosition(IntakeConstants.RETRACTED_ANGLE)
        self._position_goal = IntakeConstants.DEPLOYED_ANGLE
        self.set_position_goal(IntakeConstants.DEPLOYED_ANGLE)

    def set_position_goal(self, goal: degrees) -> None:
        if self._position_goal != goal:
            logger.info(f"Intake: Setting goal position to {goal}. currently at l/r: {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")
            self._position_goal = goal
            self._left_pid_controller.setReference(goal, SparkBase.ControlType.kPosition)
            self._right_pid_controller.setReference(goal, SparkBase.ControlType.kPosition)
            # TODO: What about velocity goal?
            #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning

    def at_deployed_angle(self, left: bool | None) -> bool:
        left_pos = self._inputs.mechanism_1_position
        right_pos = self._inputs.mechanism_2_position
        match left:
            case True:
                return left_pos <= IntakeConstants.DEPLOYED_ANGLE + IntakeConstants.TOLERANCE

            case False:
                return right_pos <= IntakeConstants.DEPLOYED_ANGLE + IntakeConstants.TOLERANCE

            case None:
                return left_pos <= IntakeConstants.DEPLOYED_ANGLE + IntakeConstants.TOLERANCE and \
                    right_pos <= IntakeConstants.DEPLOYED_ANGLE + IntakeConstants.TOLERANCE
        return True

    def at_retracted_angle(self, left: bool | None) -> bool:
        left_pos = self._inputs.mechanism_1_position
        right_pos = self._inputs.mechanism_2_position
        match left:
            case True:
                return left_pos >= IntakeConstants.RETRACTED_ANGLE - IntakeConstants.TOLERANCE

            case False:
                return right_pos >= IntakeConstants.RETRACTED_ANGLE - IntakeConstants.TOLERANCE

            case None:
                return left_pos >= IntakeConstants.RETRACTED_ANGLE - IntakeConstants.TOLERANCE and \
                    right_pos >= IntakeConstants.RETRACTED_ANGLE - IntakeConstants.TOLERANCE
        return True

    def stop(self) -> None:
        self._left_motor.stopMotor()
        self._right_motor.stopMotor()

    def periodic(self) -> None:
        LogTracer.resetOuter("IntakeSubsystem periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Intake", self._inputs)
        LogTracer.record("UpdateInputs")

        # if self._closed_loop and self._robot.isEnabled():
        #     self.set_position(self._position_goal)

        # Update visualization
        self._left_mech_base.setAngle(self._inputs.mechanism_1_position)
        self._right_mech_base.setAngle(self._inputs.mechanism_2_position)

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
        SmartDashboard.putNumber("Intake/left-speed", self._inputs.mechanism_1_speed)
        SmartDashboard.putNumber("Intake/right-position", self.right_position)
        SmartDashboard.putNumber("Intake/right-speed", self._inputs.mechanism_2_speed)
        SmartDashboard.putBoolean("Intake/closed-loop", self._closed_loop)

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
        just after the 'periodic' call and before the 'update_sim' is called.

        To unify the two uses, our call signature above has a kwargs parameter so we
        know when we are being called. Typically, we only need to support one method
        but for future simulation purposes, if called with keywords, return the amperage
        used in this interval
        """
        if self._robot.isEnabled() and self._left_sim_pivot is not None and self._left_sim_pivot is not None:
            self._left_sim_pivot.setInputVoltage(self._left_sim_motor.getAppliedOutput())
            self._left_sim_pivot.update(self._period)

            self._right_sim_pivot.setInputVoltage(self._right_sim_motor.getAppliedOutput())
            self._right_sim_pivot.update(self._period)

            # Set the simulated encoder
            self._left_sim_encoder.setPosition(self._left_sim_pivot.getAngleDegrees())
            self._right_sim_encoder.setPosition(self._right_sim_pivot.getAngleDegrees())

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._left_sim_pivot.getCurrentDraw(),
                                                           self._right_sim_pivot.getCurrentDraw()]))

    def updateInputs(self, inputs: DualMechanismIO.DualMechanismIOInputs) -> None:
        inputs.mechanism_1_connected = True   # TODO: Figure this one out
        inputs.mechanism_1_position = self._left_encoder.getPosition()
        inputs.mechanism_1_speed = self._left_encoder.getVelocity()
        inputs.mechanism_1_applied_voltage = self._left_motor.getBusVoltage()
        inputs.mechanism_1_supply_current = self._left_motor.getOutputCurrent()

        inputs.mechanism_2_connected = True   # TODO: Figure this one out
        inputs.mechanism_2_position = self._right_encoder.getPosition()
        inputs.mechanism_2_speed = self._right_encoder.getVelocity()
        inputs.mechanism_2_applied_voltage = self._right_motor.getBusVoltage()
        inputs.mechanism_2_supply_current = self._right_motor.getOutputCurrent()
        # TODO: Figure this out or drop it inputs.mechanism_torque_amps = self._motor.get

    def set_position(self, position: inches) -> None:
        """
        Set the desired encoder position. This is primarily for Autonomous mode when
        we are running with a closed loop system

        Args:
            position (rotations +/-): The desired number of rotations
        """
        # Limit to max/min
        position = max(min(position, IntakeConstants.RETRACTED_ANGLE),
                       IntakeConstants.DEPLOYED_ANGLE)

        if position != self.left_position or position != self.right_position:
            logger.info(f"Intake: Setting position to {position}")
            self._inputs.mechanism_1_position = position
            self._inputs.mechanism_2_position = position
            self._left_encoder.setPosition(position)
            self._right_encoder.setPosition(position)
            if RobotBase.isSimulation():
                self._left_sim_encoder.setPosition(position)
                self._right_sim_encoder.setPosition(position)

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        if voltage != self._left_motor.getAppliedOutput() or voltage != self._right_motor.getAppliedOutput():
            logger.info(f"Intake: Setting voltage to {voltage}")
            self._left_motor.setVoltage(voltage)
            self._right_motor.setVoltage(voltage)

    def sys_id_routine(self, subsystem: Subsystem) -> Command:
        """
        Model the behavior of the intake (for better control) by sweeping through
        the max and min heights.
        """
        def log_state(sys_id_state: State) -> None:
            match sys_id_state:
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

            Logger.recordOutput(f"Intake/SysID State", state)

        characterization_routine = SysIdRoutine(SysIdRoutine.Config(0.5, 6, 10, log_state),
                                                SysIdRoutine.Mechanism(self.set_voltage,
                                                                       (lambda _: None),
                                                                       subsystem,
                                                                       "Climber"))
        return cmd.sequence(
            cmd.runOnce(lambda: self.set_closed_loop(False), self),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kForward).until(
                lambda: self.at_retracted_angle(None)),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.at_deployed_angle(None)),
            characterization_routine.dynamic(SysIdRoutine.Direction.kForward).until(
                lambda: self.at_retracted_angle(None)),
            characterization_routine.dynamic(SysIdRoutine.Direction.kReverse).until(
                lambda: self.at_deployed_angle(None)),
            cmd.runOnce(lambda: self.set_closed_loop(True), self),
        )
