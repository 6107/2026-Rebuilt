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
from typing import Tuple

from commands2 import cmd, Subsystem
from commands2.button import Trigger
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from rev import ClosedLoopSlot, PersistMode, ResetMode, REVLibError, SparkBase, SparkClosedLoopController, SparkFlex, \
    SparkFlexConfig, SparkFlexSim, SparkRelativeEncoder, SparkRelativeEncoderSim
from wpilib import Color, Color8Bit, Mechanism2d, RobotBase, RobotController, SendableChooser, SmartDashboard
from wpilib.simulation import BatterySim, RoboRioSim, SingleJointedArmSim
from wpilib.sysid import State
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, degrees, degrees_per_second, degreesToRadians, inches, inchesToMeters, kilograms, \
    meters, radians, revolutions_per_minute, seconds, volts

from lib_6107.pykit.LoggedMechanism2d import LoggedMechanism2d
from lib_6107.pykit.LoggedMechanismLigament2d import LoggedMechanismLigament2d
from lib_6107.subsystems.pykit.dual_mechanism_io import DualMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


class PivotConstants:
    TARGET_RPM: revolutions_per_minute = 10
    PROPORTIONAL_COEFFICIENT = 0#  10  # 1e-2  # kP
    INTEGRAL_COEFFICIENT = 0       # 1e-5  # kI
    DERIVATIVE_COEFFICIENT = 0 #100   # 1e-2  # kD
    LIMIT_CURRENT: amperes = 30

    MAX_RPM: revolutions_per_minute = 6784
    GEAR_RATIO = 1.0  # TODO: Need more torque. Get working in Rev Client 2.0 first and transfer numbers here
    SPOOL_DIAMETER: meters = 1.0    # TODO: Use an algorythm to compensate for cord already spooled in
    DEPLOYED_ANGLE: degrees = 90.0  # This is straight forward since our encoder is set to zero on power up position
    RETRACTED_ANGLE: degrees = 0.0  # This is straight up.
    TOLERANCE: degrees = 2.0

    PIVOT_LEFT_ROOT_X: meters = inchesToMeters(.5)  # Bottom left corner of robot is (0, 0)
    PIVOT_RIGHT_ROOT_X: meters = inchesToMeters(24.5)
    PIVOT_ROOT_Y: meters = inchesToMeters(24.5)
    PIVOT_BASE_LENGTH: meters = inchesToMeters(8.0)  # TODO: Validate all this locations/lengths

    PIVOT_LENGTH: meters = inchesToMeters(8.0)
    PIVOT_MASS: kilograms = 1.0  # TODO: Verify

@autologgable_output
class RevIntakePivot(Subsystem, DualMechanismIO):
    """
    Intake Pivot Motor.

    This subsystem has two Rev Robotics NEO Vortex motors and is using the
    internal encoder. Gear Ratio is 1:1. Encoder conversion factors are set up
    to count in degrees. So when you input a setpoint to the PID, that value is
    the angle (degrees). Positive moves the pivot forward/down. Setting back to
    zero returns it to the zero-point (see note below).

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

    def __init__(self, container: 'RobotContainer',
                 can_left_device_id: int, can_right_device_id: int,
                 left_inverted: bool, right_inverted: bool) -> None:
        Subsystem.__init__(self)
        DualMechanismIO.__init__(self, "IntakePivot")

        # General attributes
        self.setName("IntakePivot")
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
        self._left_motor = SparkFlex(self._left_device_id, SparkFlex.MotorType.kBrushless)
        l_status = try_until_ok("Left Intake", 5,
                                lambda: self._left_motor.configure(self._motor_config(self._left_inverted),
                                                                   ResetMode.kResetSafeParameters,
                                                                   PersistMode.kNoPersistParameters))

        self._right_motor = SparkFlex(self._right_device_id, SparkFlex.MotorType.kBrushless)
        r_status = try_until_ok("Right Intake", 5,
                                lambda: self._right_motor.configure(self._motor_config(self._right_inverted),
                                                                    ResetMode.kResetSafeParameters,
                                                                    PersistMode.kNoPersistParameters))

        # Check if the device was successfully configured and can be reached over the
        # CAN bus.
        self._l_is_connected, self._r_is_connected = self._check_is_connected(l_status, r_status)

        # Set up the encoders
        self._left_encoder: SparkRelativeEncoder = self._left_motor.getEncoder()
        self._right_encoder: SparkRelativeEncoder = self._right_motor.getEncoder()

        logger.info(
            f"Intake Pivot. At startup, encoder currently at {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")

        # Support simulation
        self._left_sim_initial = None
        self._right_sim_initial = None

        if RobotBase.isSimulation():
            gearbox = DCMotor.NEO(1)
            self._left_sim_motor = SparkFlexSim(self._left_motor, gearbox)
            self._right_sim_motor = SparkFlexSim(self._right_motor, gearbox)

            self._left_sim_encoder = SparkRelativeEncoderSim(self._left_motor)
            self._right_sim_encoder = SparkRelativeEncoderSim(self._right_motor)

            moi = SingleJointedArmSim.estimateMOI(PivotConstants.PIVOT_LENGTH, PivotConstants.PIVOT_MASS)

            self._left_sim_pivot = SingleJointedArmSim(gearbox,
                                                       PivotConstants.GEAR_RATIO,
                                                       moi,
                                                       PivotConstants.PIVOT_LENGTH,
                                                       self._adjust_intake_angle_radians(
                                                           PivotConstants.DEPLOYED_ANGLE),  # Min Angle
                                                       self._adjust_intake_angle_radians(
                                                           PivotConstants.RETRACTED_ANGLE),  # Max Angle
                                                       True,
                                                       self._adjust_intake_angle_radians(
                                                           PivotConstants.RETRACTED_ANGLE))  # Starting Angle
            self._right_sim_pivot = SingleJointedArmSim(gearbox,
                                                        PivotConstants.GEAR_RATIO,
                                                        moi,
                                                        PivotConstants.PIVOT_LENGTH,
                                                        self._adjust_intake_angle_radians(
                                                            PivotConstants.DEPLOYED_ANGLE),
                                                        self._adjust_intake_angle_radians(
                                                            PivotConstants.RETRACTED_ANGLE),
                                                        True,
                                                        self._adjust_intake_angle_radians(
                                                            PivotConstants.RETRACTED_ANGLE))

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._left_pid_controller: SparkClosedLoopController = self._left_motor.getClosedLoopController()
        self._right_pid_controller: SparkClosedLoopController = self._right_motor.getClosedLoopController()

        logger.info(
            f"Intake Pivot. At startup, pid setpoints are {self._left_pid_controller.getSetpoint()}/{self._right_pid_controller.getSetpoint()}")

        self._left_pid_controller.setSetpoint(0, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
        self._right_pid_controller.setSetpoint(0, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
        # TODO: What about velocity goal?

        # The critical attributes/properties for operation
        self._position_goal: degrees = 0.0
        self._applied_voltage: volts  = 0.0

        #####################################
        # Visualization support
        left_mech_2d = Mechanism2d(inchesToMeters(20), inchesToMeters(50))
        mech_root = left_mech_2d.getRoot("Left Pivot Root",
                                         PivotConstants.PIVOT_LEFT_ROOT_X,
                                         PivotConstants.PIVOT_ROOT_Y)

        angle = self._get_rel_angle(self._adjust_intake_angle(PivotConstants.RETRACTED_ANGLE), 90)

        self._left_mech_base = mech_root.appendLigament("Left Pivot Arm",
                                                        PivotConstants.PIVOT_BASE_LENGTH,
                                                        angle,
                                                        color=Color8Bit(Color.kBlue))

        right_mech_2d = LoggedMechanism2d(inchesToMeters(20), inchesToMeters(50))
        mech_root = right_mech_2d.getRoot("Right Pivot Root",
                                          PivotConstants.PIVOT_RIGHT_ROOT_X,
                                          PivotConstants.PIVOT_ROOT_Y)
        self._right_mech_base = LoggedMechanismLigament2d("Right Pivot Base",
                                                          PivotConstants.PIVOT_BASE_LENGTH,
                                                          angle,
                                                          color=Color8Bit(Color.kBlue))
        mech_root.append(self._right_mech_base)

        SmartDashboard.putData("Left-Pivot", left_mech_2d)
        SmartDashboard.putData("Right-Pivot", right_mech_2d)

        # TODO: Remove following once all works
        # self._enable_chooser = LoggedDashboardChooser("Intake Enabled")
        self._enable_chooser = SendableChooser()
        self._enable_chooser.addOption("True", True)
        self._enable_chooser.setDefaultOption("False", False)

        if isinstance(self._enable_chooser, SendableChooser):
            SmartDashboard.putData("Intake Enabled", self._enable_chooser)

        elif isinstance(self._enable_chooser, LoggedDashboardChooser):
            pass

    def _get_rel_angle(self, target_abs: degrees, parent_abs: degrees) -> degrees:
        """
        Calculates the relative angle needed for a ligament given the
        desired absolute angle and the parent's absolute angle.
        """
        return target_abs - parent_abs

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self._enable_chooser.getSelected() is True

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkFlexConfig:
        """
        Motor config for the intake pivot. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        config = (SparkFlexConfig().
                  inverted(inverted).
                  smartCurrentLimit(PivotConstants.LIMIT_CURRENT).
                  setIdleMode(SparkFlexConfig.IdleMode.kBrake)
                  )
        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

        # Set the position conversion factor (e.g., to convert rotations to degrees)
        # The native unit is rotations. So use 360.0 as conversion factor to get degrees
        deploy_degrees_per_motor_rotation = 360 * PivotConstants.GEAR_RATIO
        config.encoder.positionConversionFactor(deploy_degrees_per_motor_rotation)

        # Set the velocity conversion factor (e.g., to convert RPM to degrees/second)
        # The native unit is RPM. So use:  (360 degrees/revolution) / (60 seconds/minute) = 6
        config.encoder.velocityConversionFactor(deploy_degrees_per_motor_rotation / 60)

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
            pidf(p=PivotConstants.PROPORTIONAL_COEFFICIENT,  # Slot 0 for position control
                 i=PivotConstants.INTEGRAL_COEFFICIENT,
                 d=PivotConstants.DERIVATIVE_COEFFICIENT,
                 ff=0,
                 slot=slot0)
            .outputRange(-1, 1)
            #   .feedForward.kG(0.05)
        )
        # TODO: For control over acceleration and velocity, use maxMotion on slot 1
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/maxmotion-position-control
        #
        # CruiseVelocity -> Set the cruise velocity for the MAXMotion mode of the controller
        #                   for a specific closed loop slot. Natively, the units are in RPM
        #                   but will be affected by the velocity conversion factor.
        #
        #  Max RPM is at 12V and we should be lower if the outputRange above is in play
        #
        crank_max_dps = PivotConstants.MAX_RPM * deploy_degrees_per_motor_rotation / 60
        slot1 = ClosedLoopSlot(ClosedLoopSlot.kSlot1)
        # (
        #     config.closedLoop.
        #     pidf(p=1e-5,
        #          i=0,
        #          d=0,
        #          ff=1 / crank_max_dps,
        #          slot=slot1)
        #     .maxMotion.cruiseVelocity(250, slot=slot1)
        #     .maxAcceleration(500, slot=slot1)
        #     .allowedClosedLoopError(0, slot=slot1)
        # )
        # TODO: review the following again
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/feed-forward-control
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning
        #       https://docs.revrobotics.com/revlib/spark/closed-loop/position-control-mode

        return config

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        return self._l_is_connected and self._r_is_connected  # Need both to work

    def _check_is_connected(self, l_status: REVLibError | None, r_status: REVLibError | None) -> Tuple[bool, bool]:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        l_version = self._left_motor.getFirmwareVersion()
        r_version = self._right_motor.getFirmwareVersion()

        logger.info(f"{self.getName()} firmware versions: {l_version}/{r_version}")

        l_ok = (l_version != 0 and (l_status is None or l_status == REVLibError.kOk))
        r_ok = (r_version != 0 and (r_status is None or r_status == REVLibError.kOk))

        if not l_ok:
            logger.warning(f"{self.getName()} (left) firmware version: {l_version}, status: {l_status}")

        if not r_ok:
            logger.warning(f"{self.getName()} (right) firmware version: {r_version}, status: {r_status}")

        return l_ok, r_ok

    @property
    def angle(self) -> degrees:
        return self._left_encoder.getPosition()

    @property
    def angular_velocity(self) -> degrees_per_second:
        return self._left_encoder.getVelocity()

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

        self._position_goal = 0
        raise NotImplementedError("Reset command was called but is not supported at this time")
        #self._left_encoder.setPosition(0.0)
        #self._right_encoder.setPosition(0.0)
        self._left_mech_base.setAngle(PivotConstants.RETRACTED_ANGLE)
        self._right_mech_base.setAngle(PivotConstants.RETRACTED_ANGLE)
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
        logger.info(f"Intake: Pivot up. Position goal before command is: {self._position_goal}")

        self.set_position_goal(PivotConstants.RETRACTED_ANGLE)

    def pivot_tweak_up(self, increment: degrees = 1.0)-> None:
        self.set_position_goal(self._position_goal + increment)

    def pivot_tweak_down(self, increment: degrees = 1.0)-> None:
        self.set_position_goal(self._position_goal - increment)

    def pivot_down(self):
        # Set encoders to 90 degrees and go down
        logger.info(f"Intake: Pivot down, currently at l/r: {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")
        logger.info(f"Intake: Pivot down. Position goal before command is: {self._position_goal}")

        self.set_position_goal(PivotConstants.DEPLOYED_ANGLE)

    def set_position_goal(self, goal: degrees) -> None:
        if self._position_goal != goal:
            logger.info(f"Intake: Setting goal position to {goal}. currently at l/r: {self._left_encoder.getPosition()}/{self._right_encoder.getPosition()}")
            logger.info(f"Intake: current PID controller setpoint before command is: {self._left_pid_controller.getSetpoint()}/{self._right_pid_controller.getSetpoint()}")

            self._position_goal = goal

            self._left_pid_controller.setSetpoint(goal, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
            self._right_pid_controller.setSetpoint(goal, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
            # TODO: What about velocity goal?
            #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning

    def at_deployed_angle(self, left: bool | None) -> bool:
        left_pos = self._inputs.mechanism_1_position
        right_pos = self._inputs.mechanism_2_position
        match left:
            case True:
                return left_pos <= PivotConstants.DEPLOYED_ANGLE + PivotConstants.TOLERANCE

            case False:
                return right_pos <= PivotConstants.DEPLOYED_ANGLE + PivotConstants.TOLERANCE

            case None:
                return left_pos <= PivotConstants.DEPLOYED_ANGLE + PivotConstants.TOLERANCE and \
                    right_pos <= PivotConstants.DEPLOYED_ANGLE + PivotConstants.TOLERANCE
        return True

    def at_retracted_angle(self, left: bool | None) -> bool:
        left_pos = self._inputs.mechanism_1_position
        right_pos = self._inputs.mechanism_2_position
        match left:
            case True:
                return left_pos >= PivotConstants.RETRACTED_ANGLE - PivotConstants.TOLERANCE

            case False:
                return right_pos >= PivotConstants.RETRACTED_ANGLE - PivotConstants.TOLERANCE

            case None:
                return left_pos >= PivotConstants.RETRACTED_ANGLE - PivotConstants.TOLERANCE and \
                    right_pos >= PivotConstants.RETRACTED_ANGLE - PivotConstants.TOLERANCE
        return True

    def stop(self) -> None:
        logger.info(f"Intake Pivot: Stop command was called")
        self._left_motor.stopMotor()
        self._right_motor.stopMotor()

    def periodic(self) -> None:
        LogTracer.resetOuter("Intake Pivot periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Intake Pivot", self._inputs)
        LogTracer.record("UpdateInputs")

        # Update visualization
        self._left_mech_base.setAngle(self._adjust_intake_angle(self._inputs.mechanism_1_position))
        self._right_mech_base.setAngle(self._adjust_intake_angle(self._inputs.mechanism_2_position))

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Intake/Pivot/goal", self._position_goal)
        Logger.recordOutput("Intake/Pivot/closed-loop", self._closed_loop)
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
        SmartDashboard.putNumber("Intake/Pivot/goal", self._position_goal)
        SmartDashboard.putBoolean("Intake/Pivot/closed-loop", self._closed_loop)
        SmartDashboard.putNumber("Intake/Pivot/left-position", self.left_position)
        SmartDashboard.putNumber("Intake/Pivot/left-speed", self._inputs.mechanism_1_speed)
        SmartDashboard.putNumber("Intake/Pivot/right-position", self.right_position)
        SmartDashboard.putNumber("Intake/Pivot/right-speed", self._inputs.mechanism_2_speed)

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
        if self._robot.isEnabled() and self._left_sim_pivot is not None and self._left_sim_pivot is not None:
            left_output = self._left_sim_motor.getAppliedOutput()
            right_output = self._right_sim_motor.getAppliedOutput()

            # First pass, set the encoder positions to the initial setting
            if self._left_sim_initial is None:
                self._left_encoder.setPosition(PivotConstants.RETRACTED_ANGLE)
                self._left_sim_initial = self._left_encoder.getPosition()

            if self._right_sim_initial is None:
                self._right_encoder.setPosition(PivotConstants.RETRACTED_ANGLE)
                self._right_sim_initial = self._right_encoder.getPosition()

            input_voltage = RobotController.getInputVoltage()  # TODO: Can we use BatterySim?
            left_applied_output = left_output * input_voltage
            right_applied_output = right_output * input_voltage

            # TODO: Can we use BatterySim?
            self._left_sim_motor.iterate(left_applied_output, input_voltage, self._period)
            self._right_sim_motor.iterate(right_applied_output, input_voltage, self._period)

            self._left_sim_pivot.setInputVoltage(left_applied_output)
            self._left_sim_pivot.update(self._period)

            self._right_sim_pivot.setInputVoltage(right_applied_output)
            self._right_sim_pivot.update(self._period)

            # Set the simulated encoder
            # self._left_sim_encoder.setPosition(self._left_sim_pivot.getAngleDegrees())
            # self._right_sim_encoder.setPosition(self._right_sim_pivot.getAngleDegrees())
            self._left_encoder.setPosition(self._left_sim_pivot.getAngleDegrees())
            self._right_encoder.setPosition(self._right_sim_pivot.getAngleDegrees())

            pivot = self._left_sim_pivot.getAngleDegrees()

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._left_sim_pivot.getCurrentDraw(),
                                                           self._right_sim_pivot.getCurrentDraw()]))

    def updateInputs(self, inputs: DualMechanismIO.DualMechanismIOInputs) -> None:
        inputs.mechanism_1_connected = self._l_is_connected
        inputs.mechanism_1_position = self._adjust_intake_angle(self._left_encoder.getPosition())
        inputs.mechanism_1_speed = self._left_encoder.getVelocity()
        inputs.mechanism_1_applied_voltage = self._left_motor.getBusVoltage()
        inputs.mechanism_1_supply_current = self._left_motor.getOutputCurrent()

        inputs.mechanism_2_connected = self._r_is_connected
        inputs.mechanism_2_position = self._adjust_intake_angle(self._right_encoder.getPosition())
        inputs.mechanism_2_speed = self._right_encoder.getVelocity()
        inputs.mechanism_2_applied_voltage = self._right_motor.getBusVoltage()
        inputs.mechanism_2_supply_current = self._right_motor.getOutputCurrent()
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

    def set_position(self, position: inches) -> None:
        """
        Set the desired encoder position. This is primarily for Autonomous mode when
        we are running with a closed loop system

        Args:
            position (rotations +/-): The desired number of rotations
        """
        # Limit to max/min
        position = max(min(position, PivotConstants.RETRACTED_ANGLE),
                       PivotConstants.DEPLOYED_ANGLE)

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

        # TODO: Add to non-competition AUTO chooser
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
                                                                       "Intake Pivot"))
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
