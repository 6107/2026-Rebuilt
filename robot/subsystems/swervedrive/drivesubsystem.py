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

import json
import logging
import math
import os
from collections import OrderedDict
from typing import Callable
from typing import List, Optional, Sequence, Tuple

from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
from phoenix6 import SignalLogger, swerve, units, utils
from phoenix6.swerve.requests import FieldCentric, RobotCentric
from phoenix6.swerve.swerve_module import SwerveModule
from pykit.autolog import autolog_output, autologgable_output
from pykit.logger import Logger
from wpilib import DriverStation, Notifier, RobotController
from wpilib import Field2d, RobotBase, SmartDashboard
from wpilib import getDeployDirectory
from wpilib.sysid import SysIdRoutineLog
from wpimath.filter import SlewRateLimiter
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.kinematics import ChassisSpeeds, SwerveModulePosition, SwerveModuleState
from wpimath.units import degrees, meters, meters_per_second, radians_per_second, rotationsToRadians

from constants import GYRO_REVERSED, JOYSTICK_DEADBAND, MAX_SPEED, USE_PYKIT, WHEEL_CIRCUMFERENCE, WHEEL_RADIUS
from field.field_2026 import BLUE_TEST_POSE, FIELD_X_SIZE, FIELD_Y_SIZE, RED_TEST_POSE
from generated.tuner_constants import TunerSwerveDrivetrain
from lib_6107.subsystems.gyro.gyro import Gyro
from lib_6107.subsystems.pykit.gyro_io import GyroIO
from lib_6107.subsystems.pykit.swervedrive_io import SwerveDriveIO
from subsystems.swervedrive.constants import DriveConstants

try:
    import navx

    NAVX_SUPPORTED = True
except ImportError:
    NAVX_SUPPORTED = False

# TODO: This value needs to be tested. Perform the following on a real robot
#
# Measuring Overshoot
# Implement a Control Loop:
#   You cannot simply apply constant power until the target angle is reached, as the robot needs time to decelerate
#   and will inevitably overshoot. A simple Proportional (P) loop is the standard starting point for FRC teams.
#   The motor power is made proportional to the difference between the target angle and the current angle.
#
#      Formula (simplified P-loop): motorPower = (targetAngle - currentAngle) * kP
#      kP is a constant you tune to get the desired performance.
#
# Log Data:
#   Use your FRC development environment (e.g., WPILib) to log the robot's current gyro angle and the target
#   angle to a file or SmartDashboard/Shuffleboard.
#
# Perform a Test Turn:
#   Command your robot to turn to a specific, significant angle (e.g., 90 degrees) using the P-loop,
#   and log the data during the process.
#
# Analyze the Data:
#   After the test, view the logged data in a graph or spreadsheet.
# Target Angle:
#   The desired final angle (e.g., 90 degrees).
#
# Peak Angle:
#   The maximum angle the robot reaches during the turn before it starts correcting back towards the target.
#
# Calculate Overshoot:
#   The difference between the peak angle and the target angle is the overshoot.
#
# Overshoot = Peak Angle - Target Angle
#
# Correcting Overshoot
#
# The primary method for reducing overshoot is tuning your control loop.
#
# Adjust kP:
#   If your robot consistently overshoots significantly, your kP value is likely too high. Lowering it
#   will make the turn slower but more accurate.
#
# Add Derivative (D) control:
#   Implementing a full PID loop can help. The derivative term (kD) dampens the system by applying a
#   counter-force based on how fast the error is changing (i.e., the robot's turn rate), which helps
#   slow the robot down as it approaches the target.
#
# Slow Down Turns:
#   As a simple fix, reducing the maximum motor power or speed used for turns will also reduce overshoot,
#   though it makes the robot slower overall.
#
# Ensure Proper Calibration:
#   Make sure the gyro is stationary during its initial calibration phase (when the robot code starts)
#   to minimize drift and baseline errors.
#
GYRO_OVERSHOOT_FRACTION = -3.25 / 360
# ^^ our gyro didn't overshoot, it "undershot" by 0.1 degrees in a 360 degree turn

SwerveModuleStates = Sequence[SwerveModuleState]

logger = logging.getLogger(__name__)


@autologgable_output
class DriveSubsystem(Subsystem, TunerSwerveDrivetrain):
    _SIM_LOOP_PERIOD: units.second = 0.004  # 4 ms

    _BLUE_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0)
    """Blue alliance sees forward as 0 degrees (toward red alliance wall)"""
    _RED_ALLIANCE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180)
    """Red alliance sees forward as 180 degrees (toward blue alliance wall)"""

    def __init__(self, consts, modules, container: 'RobotContainer') -> None:

        # super().__init__(hardware.TalonFX, hardware.TalonFX, hardware.CANcoder, consts, modules)
        Subsystem.__init__(self)
        TunerSwerveDrivetrain.__init__(self, consts, modules)

        self._container = container
        self._robot = container.robot
        self._physics_controller = None
        self._last_pose: Optional[Pose2d] = None

        # Camera/localizer defaults
        self.vision_odometry = False
        self.field_relative = False  # Assume robot-relative to start with

        # self.gyroOvershootFraction = 0.0
        # if not TimedCommandRobot.isSimulation():
        #     self.gyroOvershootFraction = GYRO_OVERSHOOT_FRACTION

        # The modules are created in the following order in our tuner_constants 'create_drivetrain' func
        self._swerve_modules: OrderedDict[str, SwerveModule] = OrderedDict(
            [
                ("front-left",  self.modules[0]),
                ("front-right", self.modules[1]),
                ("back-left",   self.modules[2]),
                ("back-right",  self.modules[3])
            ])

        # Some useful requests amd constants
        max_speed = MAX_SPEED
        max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        # Setting up bindings for necessary control of the Phoenix6 swerve drive platform.
        # This sets a deadband for both the speed and rotation control
        #  Use open-loop control for drive motors
        self._drive = (swerve.requests.FieldCentric().with_deadband(max_speed * JOYSTICK_DEADBAND)
                       .with_rotational_deadband(max_angular_rate * JOYSTICK_DEADBAND)
                       .with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))

        self._brake = swerve.requests.SwerveDriveBrake()

        self._point = swerve.requests.PointWheelsAt()

        self._forward_straight = (swerve.requests.RobotCentric().
                                  with_drive_request_type(swerve.SwerveModule.DriveRequestType.OPEN_LOOP_VOLTAGE))

       # The gyro/IMU sensor  # TODO: may be able to pull
        self._gyro: Optional[Gyro] = Gyro.create("Pigeon2",
                                                 GYRO_REVERSED,
                                                 inst=self.pigeon2)
        self._gyro.initialize()
        self._gyroInputs = GyroIO.GyroIOInputs()

        # TODO: Support slew rate and make adjustable. There is a parameter to 'drive()'
        #       called rate limit that currently uses the rotation & magnitude limiter
        # Slew rate filter variables and limiters for controlling lateral acceleration
        self.magLimiter = SlewRateLimiter(DriveConstants.MAGNITUDE_SLEW_RATE)
        self.rotLimiter = SlewRateLimiter(DriveConstants.ROTATIONAL_SLEW_RATE)

        self._robot_x_width = 0.4
        self._robot_y_width = 0.4
        try:
            path = os.path.join(getDeployDirectory(), 'pathplanner', 'settings.json')

            with open(path, 'r') as f:
                settings = json.loads(f.read())
                self._robot_x_width = settings.get("robotWidth", self._robot_x_width)
                self._robot_y_width = settings.get("robotWidth", self._robot_y_width)

        except FileNotFoundError:
            pass

        # Pykit support
        self._inputs = SwerveDriveIO.DriveIOInputs()

        # The next attributes are set depending on if vision is unsupported for tracking the robot pose
        self._network_table_inst = None

        # Check for any alliance change and return our initial pose
        self.pose = self._alliance_change(container.is_red_alliance,
                                          container.alliance_location)

        # Set standard deviation for vision odometry to trust vision less if
        # far away. Only takes effect if we call the addVisionMeasurement to
        # provide a pose estimation.  Values could be:
        #
        #  High Confidence (Close/Multiple Tags):  (0.1,0.1,0.1) (meters, meters, radians).
        #
        #  Low Confidence (Far/Single Tag):         (0.5,0.5,99999) effectively ignoring vision
        #                                           heading to rely on the gyro.
        #
        #  The call to 'addVisionMeasurement' can optionally have a standard deviation value, and
        #  it remains in effect until another measurement std deviation is provided. We will start
        #  with a high confidence since autonomous mode is heavily reliant on vision, and we
        #  expect to traverse the 'bump' at least twice.
        self.set_vision_measurement_std_devs((0.2, 0.2, 0.2))

        # robot_config = RobotConfig.fromGUISettings()
        #
        # # TODO: What do we need here with pathplanner and assuming pykit is also supported. Also if
        # #       we want the PPLTV Controller, should we use the AutoConstants or DriveConstants.
        # # controller = apriltag.k_pathplanner_holonomic_controller
        # controller = PPLTVController(1 / kwargs["pykit"]["Update Frequency"],
        #                              MAX_SPEED)
        #
        # # # TODO: Validate what get_relative_speeds is suppose to return in the actual call.
        # # #       it looks like it is suppose to be 4 values and not a list of 4 values (ModuleStates)
        pass
        #
        # Register for any changes in alliance before the match starts
        container.register_alliance_change_callback(self._alliance_change)

        self._sim_notifier: Notifier | None = None
        self._last_sim_time: units.second = 0.0

        self._has_applied_operator_perspective = False
        """Keep track if we've ever applied the operator perspective before or not"""

        # Swerve request to apply during path following
        self.apply_robot_speeds = swerve.requests.ApplyRobotSpeeds()

        # Swerve requests to apply during SysId characterization
        self._translation_characterization = swerve.requests.SysIdSwerveTranslation()
        self._steer_characterization = swerve.requests.SysIdSwerveSteerGains()
        self._rotation_characterization = swerve.requests.SysIdSwerveRotation()

        self._sys_id_routine_translation = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Reduce dynamic voltage to 4 V to prevent brownout
                stepVoltage=4.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdTranslation_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._translation_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing translation. This is used to find PID gains for the drive motors."""

        self._sys_id_routine_steer = SysIdRoutine(
            SysIdRoutine.Config(
                # Use default ramp rate (1 V/s) and timeout (10 s)
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: self.set_control(
                    self._steer_characterization.with_volts(output)
                ),
                lambda log: None,
                self,
            ),
        )
        """SysId routine for characterizing steer. This is used to find PID gains for the steer motors."""

        self._sys_id_routine_rotation = SysIdRoutine(
            SysIdRoutine.Config(
                # This is in radians per second², but SysId only supports "volts per second"
                rampRate=math.pi / 6,
                # Use dynamic voltage of 7 V
                stepVoltage=7.0,
                # Use default timeout (10 s)
                # Log state with SignalLogger class
                recordState=lambda state: SignalLogger.write_string(
                    "SysIdSteer_State", SysIdRoutineLog.stateEnumToString(state)
                )
                                          and None,
            ),
            SysIdRoutine.Mechanism(
                lambda output: (
                                   # output is actually radians per second, but SysId only supports "volts"
                                   self.set_control(
                                       self._rotation_characterization.with_rotational_rate(output)
                                   ),
                                   # also log the requested output for SysId
                                   SignalLogger.write_double("Rotational_Rate", output),
                               )
                               and None,
                lambda log: None,
                self,
            ),
        )
        """
        SysId routine for characterizing rotation.
        This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
        See the documentation of swerve.requests.SysIdSwerveRotation for info on importing the log to SysId.
        """

        self._sys_id_routine_to_apply = self._sys_id_routine_translation
        """The SysId routine to test"""

        if utils.is_simulation():
            self._start_sim_thread()

    @property
    def robot(self) -> 'MyRobot':
        return self._robot

    @property
    def container(self) -> 'RobotContainer':
        return self._container

    @property
    def drive_request(self) -> swerve.requests.FieldCentric:
        return self._drive

    @property
    def point_at_request(self) -> swerve.requests.PointWheelsAt:
        return self._point

    @property
    def forward_straight_request(self) -> RobotCentric:
        return self._forward_straight

    @property
    def brake_request(self) -> swerve.requests.SwerveDriveBrake:
        return self._brake

    def apply_request(
            self, request: Callable[[], swerve.requests.SwerveRequest]
    ) -> Command:
        """
        Returns a command that applies the specified control request to this swerve drivetrain.

        :param request: Lambda returning the request to apply
        :type request: Callable[[], swerve.requests.SwerveRequest]
        :returns: Command to run
        :rtype: Command
        """
        return self.run(lambda: self.set_control(request()))

    def sys_id_quasistatic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Quasistatic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Quasistatic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.quasistatic(direction)

    def sys_id_dynamic(self, direction: SysIdRoutine.Direction) -> Command:
        """
        Runs the SysId Dynamic test in the given direction for the routine
        specified by self.sys_id_routine_to_apply.

        :param direction: Direction of the SysId Dynamic test
        :type direction: SysIdRoutine.Direction
        :returns: Command to run
        :rtype: Command
        """
        return self._sys_id_routine_to_apply.dynamic(direction)

    def _start_sim_thread(self):
        def _sim_periodic():
            current_time = utils.get_current_time_seconds()
            delta_time = current_time - self._last_sim_time
            self._last_sim_time = current_time

            # use the measured time delta, get battery voltage from WPILib
            self.update_sim_state(delta_time, RobotController.getBatteryVoltage())

        # Run simulation at a faster rate so PID gains behave more reasonably
        self._last_sim_time = utils.get_current_time_seconds()
        self._sim_notifier = Notifier(_sim_periodic)
        self._sim_notifier.startPeriodic(self._SIM_LOOP_PERIOD)

    def add_vision_measurement(
            self,
            vision_robot_pose: Pose2d,
            timestamp: units.second,
            vision_measurement_std_devs: tuple[float, float, float] | None = None,
    ):
        """
        Adds a vision measurement to the Kalman Filter. This will correct the
        odometry pose estimate while still accounting for measurement noise.

        Note that the vision measurement standard deviations passed into this method
        will continue to apply to future measurements until a subsequent call to
        set_vision_measurement_std_devs or this method.

        :param vision_robot_pose:           The pose of the robot as measured by the vision camera.
        :type vision_robot_pose:            Pose2d
        :param timestamp:                   The timestamp of the vision measurement in seconds.
        :type timestamp:                    second
        :param vision_measurement_std_devs: Standard deviations of the vision pose measurement
                                            in the form [x, y, theta]ᵀ, with units in meters
                                            and radians.
        :type vision_measurement_std_devs:  tuple[float, float, float] | None
        """
        TunerSwerveDrivetrain.add_vision_measurement(
            self,
            vision_robot_pose,
            utils.fpga_to_current_time(timestamp),
            vision_measurement_std_devs
        )

    def sample_pose_at(self, timestamp: units.second) -> Pose2d | None:
        """
        Return the pose at a given timestamp, if the buffer is not empty.

        :param timestamp: The timestamp of the pose in seconds.
        :type timestamp: second
        :returns: The pose at the given timestamp (or None if the buffer is empty).
        :rtype: Pose2d | None
        """
        return TunerSwerveDrivetrain.sample_pose_at(self, utils.fpga_to_current_time(timestamp))

    @property
    def counter(self) -> int:
        return self._robot.counter

    @property
    def field(self) -> Field2d:
        return self._robot.field

    @property
    def gyro(self) -> Gyro:
        return self._gyro

    # ###########################################
    # # PathPlanner Support (TODO: NOT FULLY SUPPORTED YET ENABLED - COMMENTED OUT IN INIT)
    # def runClosedLoop(self, speeds: ChassisSpeeds,
    #                   _feedForwards: Optional[DriveFeedforwards] = None):
    #     wheelSpeeds = self.kinematics.toWheelSpeeds(speeds)
    #     self.runClosedLoopParameters(wheelSpeeds.left, wheelSpeeds.right)
    #
    if USE_PYKIT:
        def runClosedLoopParameters(self, left_speed: float, right_speed: float):
            from numpy import sign

            left_rad_per_s = left_speed / WHEEL_RADIUS
            right_rad_per_s = right_speed / WHEEL_RADIUS

            Logger.recordOutput("Drive/LeftSetpoint", left_rad_per_s)
            Logger.recordOutput("Drive/RightSetpoint", right_rad_per_s)

            left_ff = self.kS * sign(left_rad_per_s) + self.kV * left_rad_per_s
            right_ff = self.kS * sign(right_rad_per_s) + self.kV * right_rad_per_s

            self._inputs.setVelocity(left_rad_per_s, right_rad_per_s, left_ff, right_ff)

        def runOpenLoop(self, left_v: float, right_v: float) -> None:
            self._inputs.setVoltage(left_v, right_v)

        def sysIdQuasistatic(self, direction: SysIdRoutine.Direction):
            return self.sysid.quasistatic(direction)

        def sysIdDynamic(self, direction: SysIdRoutine.Direction):
            return self.sysid.dynamic(direction)

    def _alliance_change(self, is_red: bool, location: int) -> Pose2d:
        """
        Change in alliance occurred before match started. If simulation is
        supported, then 'physics.py' handles this.
        """
        if not RobotBase.isSimulation():
            return Pose2d(0, 0, 0)

        initial_pose = Pose2d(0, 0, Rotation2d.fromDegrees(self.gyro.yaw))
        if location in (1, 2, 3):
            # Use test subsystem settings if simulation
            initial_pose = RED_TEST_POSE[location] if is_red else BLUE_TEST_POSE[location]

        self.pose = initial_pose
        return initial_pose

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        SmartDashboard.putData("Field", self.field)
        self.gyro.dashboard_initialize()

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        if update_dash and self._last_pose is not None:
            SmartDashboard.putNumber("Drivetrain/x", self._last_pose.x)
            SmartDashboard.putNumber("Drivetrain/y", self._last_pose.y)
            SmartDashboard.putNumber("Drivetrain/heading", self._last_pose.rotation().degrees())

            self.gyro.dashboard_periodic()

    def periodic(self) -> None:
        enabled = self._robot.isEnabled()
        log_it = self._robot.counter % 20 == 0 and enabled

        # Periodically try to apply the operator perspective.
        # If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
        # This allows us to correct the perspective in case the robot code restarts mid-match.
        # Otherwise, only check and apply the operator perspective if the DS is disabled.
        # This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
        if not self._has_applied_operator_perspective or DriverStation.isDisabled():
            alliance_color = DriverStation.getAlliance()
            if alliance_color is not None:
                self.set_operator_perspective_forward(
                    self._RED_ALLIANCE_PERSPECTIVE_ROTATION
                    if alliance_color == DriverStation.Alliance.kRed
                    else self._BLUE_ALLIANCE_PERSPECTIVE_ROTATION
                )
                self._has_applied_operator_perspective = True

        self.gyro.periodic(self._gyroInputs)

        if USE_PYKIT:
            # TODO: FOLLOWING was from differential drive. change to SwerveDrive support
            #
            # TODO: Once we support PYKIT, add 'getPosition...' methods and decoreate them
            #       with the @autolog_output. See pykit example
            self.io.updateInputs(self._inputs)
            self.gyroIO.updateInputs(self._gyroInputs)

            Logger.processInputs("Drive", self._inputs)
            Logger.processInputs("Drive/Gyro", self._gyroInputs)

            if self._gyroInputs.connected:
                self.rawGyroRotation = self._gyroInputs.yawPosition
            else:
                twist = self.kinematics.toTwist2d(
                    self.getLeftPosition() - self.lastLeftPosition,
                    self.getRightPosition() - self.lastRightPosition,
                )
                self.rawGyroRotation = self.rawGyroRotation + Rotation2d(twist.dtheta)

            self.lastLeftPosition = self.getLeftPosition()
            self.lastRightPosition = self.getRightPosition()

            self.poseEstimator.update(
                self.rawGyroRotation, self.getLeftPosition(), self.getRightPosition()
            )

        # Update the odometry in the periodic block
        # TODO: For pheonix6 library, just need to pass in vision measurements
        self._last_pose = self.pose

        if self._last_pose is not None:
            self.field.setRobotPose(self._last_pose)

        # Update SmartDashboard for this subsystem
        self.dashboard_periodic()

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller

        self.gyro.sim_init(physics_controller)

    def simulationPeriodic(self, **kwargs) -> Optional[float]:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function. It is useful for updating subsystem-specific state that needs to be
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
        # For the swerve drive, we only support the 'update_sim' form of call
        if not kwargs:
            return None

        # now, tm_diff = kwargs["now"], kwargs["tm_diff"]
        amperes_used = 0.0  # TODO: Support in future

         # log_it = self._robot.counter % 20 == 0

        # Since simulation, limit it to the field of play.
        pose = self.pose

        # self.gyro.sim_yaw = pose.rotation().degrees()     # Not saving this yet.

        # Limit it to the field size (manually)
        robot_x_offset = self._robot_x_width / 2
        robot_y_offset = self._robot_y_width / 2

        x, y = pose.x, pose.y

        if x < robot_x_offset or x > FIELD_X_SIZE - robot_x_offset or \
            y < robot_y_offset or y > FIELD_Y_SIZE - robot_y_offset:
            x = min(FIELD_X_SIZE - robot_x_offset, max(robot_x_offset, x))
            y = min(FIELD_Y_SIZE - robot_y_offset, max(robot_y_offset, y))

            if x != pose.x or y != pose.y:
                self.pose = Pose2d(x, y, pose.rotation())

        return amperes_used

    @property
    def heading(self) -> Rotation2d:
        return self.pose.rotation()

    @property
    def pose(self) -> Pose2d:
        """
        Returns the currently-estimated pose of the robot.
        """
        return self.get_pose()

    @pose.setter
    def pose(self, pose: Pose2d) -> None:
        # Update the drivetrain
        self.reset_pose(pose)

    @autolog_output(key="Odometry/Robot")
    def get_pose(self) -> Pose2d:
        return self.get_state().pose

    # if USE_PYKIT:
    #     # TODO: FOLLOWING needed to support swerve drive.
    #     @autolog_output(key="Drive/LeftPosition")
    #     def getLeftPosition(self) -> float:
    #         return self._inputs.leftPositionRad * driveconstants.kWheelRadius
    #
    #     @autolog_output(key="Drive/RightPosition")
    #     def getRightPosition(self) -> float:
    #         return self._inputs.rightPositionRad * driveconstants.kWheelRadius
    #
    #     @autolog_output(key="Drive/LeftVelocity")
    #     def getLeftVelocity(self) -> float:
    #         return self._inputs.leftVelocityRadPerSec * driveconstants.kWheelRadius
    #
    #     @autolog_output(key="Drive/RightVelocity")
    #     def getRightVelocity(self) -> float:
    #         return self._inputs.rightVelocityRadPerSec * driveconstants.kWheelRadius

    def stop(self):
        if USE_PYKIT:
            self.runOpenLoop(0, 0)
        else:
            self.arcade_drive(0, 0, field_relative=True)

    def arcade_drive(self, speed: meters_per_second, rot: radians_per_second, field_relative: Optional[bool] = False,
                     assume_manual_input: bool = False) -> None:
        self.drive(speed, 0, rot, square=assume_manual_input, field_relative=field_relative)

    def rotate(self, rotation: radians_per_second) -> None:
        """
        Rotate the robot in place, without moving laterally (for example, for aiming)

        :param rotation: rotational speed
        """
        self.arcade_drive(0, rotation, field_relative=True)

    def drive(self, x_speed: meters_per_second, y_speed: meters_per_second,
              rotation: radians_per_second, field_relative: Optional[bool] = False,
              rate_limit: Optional[bool] = False, square: Optional[bool] = False) -> None:
        """
        Method to drive the robot using joystick info.

        :param x_speed:        Speed of the robot in the x direction (forward).
        :param y_speed:        Speed of the robot in the y direction (sideways).
        :param rotation:       Angular rate of the robot.
        :param field_relative: Whether the provided x and y speeds are relative to the field.
        :param rate_limit:     Whether to enable rate limiting for smoother control.
        :param square:         Whether to square the inputs (useful for manual control)
        """
        if square:
            raise NotImplementedError("TODO: Look at 2025 code if you need this")

        if rate_limit:
            raise NotImplementedError("TODO: Look at 2025 code if you need this")

        # We scale our speed down during development
        # scaler = self._container.chosenLimiter.getSelected()
        # if not isinstance(scaler, (int, float)) or scaler < 0.0 or scaler > 1.0:
        #     logger.info(f"Invalid drive rate limiter: '{scaler}")
        #     scaler = 0.1
        #
        # # Convert the commanded speeds into the correct units for the drivetrain
        # self.xSpeedDelivered = xSpeedCommanded * MAX_SPEED * scaler
        # self.ySpeedDelivered = ySpeedCommanded * MAX_SPEED * scaler
        # self.rotDelivered = self.currentRotation * DriveConstants.MAX_ANGULAR_SPEED * scaler

        scale_factor = self.drive_scale_factor
        max_speed = self._container.max_speed   # This is already adjusted by the scaling factor

        def adjust(rate: meters_per_second) -> meters_per_second:
            return min(max_speed, rate * scale_factor)

        if field_relative:
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed,
                                                           rotation, self.gyro.heading)

            request = (FieldCentric().with_velocity_x(adjust(speeds.vx)).
                                      with_velocity_y(adjust(speeds.vy)).
                                      with_rotational_rate(speeds.omega * scale_factor))
        else:
            speeds = ChassisSpeeds(x_speed, y_speed, rotation)
            request = (RobotCentric().with_velocity_x(adjust(speeds.vx)).
                                      with_velocity_y(adjust(speeds.vy)).
                                      with_rotational_rate(speeds.omega * scale_factor))
        self.set_control(request)

    def set_module_states(self, module_states: SwerveModuleStates) -> None:
        """
        Set the module state. The modules in the sequence are always provided (and used) in the
        order when the SwerveDriveKinematics object was created. Convention is typically:

           (Front Left, Front Right, Rear Left, Rear Right)
        """
        from phoenix6.controls import VelocityTorqueCurrentFOC, PositionVoltage

        angle_setter: PositionVoltage = PositionVoltage(0, 0, enable_foc=False, override_brake_dur_neutral=False)
        velocity_setter: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0, 0)
        ticks_per_revolution = 4096
        # wheel_radius: meters = WHEEL_RADIUS
        wheel_circumference: meters = WHEEL_CIRCUMFERENCE

        for index, module in enumerate(self.modules):
            drive_motor = module.drive_motor
            steer_motor = module.steer_motor

            state: SwerveModuleState = module_states[index]
            state.optimize(module.get_current_state().angle)

            # Convert linear speed (m/s) to wheel rotations per second (RPS)
            wheel_rps = state.speed / wheel_circumference
            angle_to_set = (state.angle.degrees() / 360.0) * ticks_per_revolution

            steer_motor.set_control(angle_setter.with_position(angle_to_set))
            drive_motor.set_control(velocity_setter.with_velocity(wheel_rps))

    @property
    def drive_scale_factor(self) -> float:
        # We scale our speed down during development
        scaler = 1.0

        try:
            scaler = self._container._limit_chooser.getSelected()
            if not isinstance(scaler, (int, float)):
                logger.error(f"Invalid drive rate limiter: '{scaler}")
                scaler = 0.1
            else:
                scaler = max(min(scaler, 1.0), 0.0)

        except Exception as _e:
            pass

        return scaler


    def set_motor_brake(self, brake: bool) -> None:
        # TODO: Need to actually set the IdleMode to 'brake' since this
        #       would be useful on an incline as well.   The container has a _brake command.  See what it dies
        if brake:
            self.stop()
            self.apply_request(lambda: self.brake_request)
        else:
            self.set_straight()

    def set_straight(self) -> None:
        """
        Sets the wheels straight so we can push the robot.
        """
        self.apply_request(lambda: self.point_at_request.with_module_direction(Rotation2d(0.0)))

    def get_module_positions(self) -> Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]:
        pos = [m.getPosition() for m in self._swerve_modules.values()]
        return pos[0], pos[1], pos[2], pos[3]

    def get_angle(self) -> degrees:
        return self.get_pose.rotation().degrees()

    def get_desired_swerve_module_states(self) -> List[SwerveModuleState]:
        """
        what it says on the wrapper; it's for physics.py because I don't like relying on an NT entry
        to communicate between them (it's less clear what the NT entry is there for, I think) LHACK 1/12/25
        """
        return [module.getDesiredState() for module in self._swerve_modules.values()]

    ##########################################################
    # TODO: All the following are related to team 2429 and pathplanner. These have not been tested and
    #       we may need to refactor the 'drive()' method above

    #  -------------  THINGS PATHPLANNER NEEDS  - added for pathplanner 20230218 CJH

    # def follow_pathplanner_trajectory_command(self, trajectory:PathPlannerTrajectory, is_first_path:bool):
    #     #from pathplannerlib.path import PathPlannerPath
    #     #from pathplannerlib.commands import FollowPathWithEvents, FollowPathHolonomic
    #     #from pathplannerlib.config import HolonomicPathFollowerConfig, ReplanningConfig, PIDConstants

    #     # copy of pathplannerlib's method for returning a swervecommand, with an optional odometry reset
    #     # using the first pose of the trajectory
    #     if is_first_path:
    #         reset_cmd = commands2.InstantCommand(lambda: self.pose = trajectory.getInitialTargetHolonomicPose()))
    #     else:
    #         reset_cmd = commands2.InstantCommand()

    #     # useful stuff controller.PPHolonomicDriveController, controller.PIDController, auto.FollowPathHolonomic
    #     swerve_controller_cmd = None

    #     cmd = commands2.SequentialCommandGroup(reset_cmd, swerve_controller_cmd)

    #     return cmd

    #  END PATHPLANNER STUFF
