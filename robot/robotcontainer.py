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
import platform
import time
from typing import List, Optional, Callable, Dict, Any

from commands2.sysid import SysIdRoutine

from wpimath.geometry import Rotation2d
from commands2.button import Trigger, CommandXboxController
from commands2 import Subsystem, Command, RunCommand, InstantCommand, cmd, button
from phoenix6 import swerve
from wpilib import RobotBase, XboxController, SmartDashboard, SendableChooser, Field2d, DriverStation
from wpimath.units import rotationsToRadians, meters_per_second, radians_per_second, degreesToRadians

import constants

from autonomous import pathplanner
from generated.tuner_constants import TunerConstants
from subsystems.constants import FRONT_CAMERA_TYPE, CAMERA_TYPE_LIMELIGHT, \
    CAMERA_TYPE_PHOTONVISION, FRONT_CAMERA_POSE_AND_HEADING, REAR_CAMERA_TYPE
from lib_6107.util.phoenix6_telemetry import Telemetry
from lib_6107.commands.camera.follow_object import FollowObject, StopWhen
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.reset_xy import ResetXY
from lib_6107.commands.drivetrain.trajectory import SwerveTrajectory, JerkyTrajectory
from lib_6107.constants import DEFAULT_ROBOT_FREQUENCY
from lib_6107.subsystems.limelight_camera import LimelightCamera
from lib_6107.subsystems.limelight_localizer import LimelightLocalizer

try:
    from lib_6107.subsystems.photon_localizer import PhotonLocalizer, PHOTONLIB_SUPPORTED
    from lib_6107.subsystems.photonvision_camera import PhotonVisionCamera
    PHOTONLIB_SUPPORTED=True

except ImportError:
    PHOTONLIB_SUPPORTED = False

# TODO: path planner stuff needed?

logger = logging.getLogger(__name__)

class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    def __init__(self, robot: 'MyRobot') -> None:
        # The robot's subsystems
        logger.debug("*** called container __init__")
        self.start_time = time.time()
        self.robot = robot
        self.simulation = RobotBase.isSimulation()

        # Phoenix6 max settings and telemetry support. During the actual drive command or the
        # arcade_drive function, we apply any scale factor to limit speed
        self._max_speed: meters_per_second = constants.MAX_SPEED  # speed_at_12_volts desired top speed
        self._max_angular_rate: radians_per_second = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity
        self._logger = Telemetry(self._max_speed)

        # Alliance support
        self._is_red_alliance: bool = False  # Coordinate system based off of blue being to the 'left'
        self._alliance_location: int = 1  # Valid numbers are 1, 2, 3
        self._alliance_change_callbacks: List[Callable[[bool, int], None]] = []

        # The driver's controller
        self.driver_controller = CommandXboxController(constants.DRIVER_CONTROLLER_PORT)

        # TODO: WPILib has a wpimath.fileter.SlewRateLimiter to make joystick more gentle.  Look into this
        #       See https://github.com/robotpy/examples/blob/main/SwerveBot/robot.py for an example

        # Shooter's controller
        self.controller_shooter: XboxController = XboxController(constants.SHOOTER_CONTROLLER_PORT)  # On USB-port

        ########################################################
        # Subsystem initialization
        #
        #   VISION
        #
        camera_subsystems = []
        self.localizer = None
        self.vision_odometry = False
        period = robot.getPeriod() or DEFAULT_ROBOT_FREQUENCY

        drive_kwargs: Dict[str, Any] = {
            "pykit": {
                "Update Frequency": 1.0 / period
            }
        }
        if FRONT_CAMERA_TYPE == CAMERA_TYPE_PHOTONVISION and PHOTONLIB_SUPPORTED:
            self.front_camera = PhotonVisionCamera(self, name="front-camera")

        elif FRONT_CAMERA_TYPE == CAMERA_TYPE_LIMELIGHT:
            self.front_camera = LimelightCamera(self, name="front-camera")

        else:
            self.front_camera = None

        if REAR_CAMERA_TYPE == CAMERA_TYPE_PHOTONVISION and PHOTONLIB_SUPPORTED:
            self.rear_camera = PhotonVisionCamera(self, name="rear-camera")

        elif REAR_CAMERA_TYPE == CAMERA_TYPE_LIMELIGHT:
            self.rear_camera = LimelightCamera(self, name="rear-camera")

        else:
            self.rear_camera = None

        if self.front_camera or self.rear_camera:
            # Primary camera
            drive_kwargs["Cameras"] = {}

            if self.front_camera:
                drive_kwargs["Cameras"]["Front"] = {
                    "Camera": self.front_camera
                }
                camera_subsystems.append(self.front_camera)

            if self.rear_camera:
                drive_kwargs["Cameras"]["Rear"] = {
                    "Camera": self.rear_camera
                }
                camera_subsystems.append(self.rear_camera)

        # self.robot_drive = DriveSubsystem(self, **drive_kwargs)
        self.robot_drive = TunerConstants.create_drivetrain(self, **drive_kwargs)

        # Init the Auto chooser.  PathPlanner init will fill in our choices
        self._auto_chooser = pathplanner.configure_auto_builder(self.robot_drive, self,"")

        if FRONT_CAMERA_TYPE == CAMERA_TYPE_LIMELIGHT:
            # TODO: Make pose and heading below as constants
            self.localizer = LimelightLocalizer(self, self.robot_drive)
            self.localizer.addCamera(self.front_camera,
                                     cameraPoseOnRobot=FRONT_CAMERA_POSE_AND_HEADING["Pose"],
                                     cameraHeadingOnRobot=FRONT_CAMERA_POSE_AND_HEADING["Heading"])
            drive_kwargs["Cameras"]["Front"]["Localizer"] = self.localizer
            camera_subsystems.append(self.localizer)

        elif FRONT_CAMERA_TYPE == CAMERA_TYPE_PHOTONVISION and PHOTONLIB_SUPPORTED:
            self.localizer = PhotonLocalizer(self, self.robot_drive, "2025-reefscape.json")
            drive_kwargs["Cameras"]["Front"]["Localizer"] = self.localizer

        else:
            self.localizer = None

        # Now save off our subsystems. The robot core code will already call the periodic() function
        # as needed, but having our own list (iterated in order) allows us to move much of
        # the other subsystem 'tasks' into a generic loop.

        self.subsystems: List[Subsystem] = camera_subsystems + [
            self.robot_drive,
        ]
        ########################################################
        # Configure the button bindings
        for controller, is_driver in ((self.driver_controller, True),
                                      (self.controller_shooter, False)):
            if isinstance(controller, CommandXboxController):
                self.configure_button_bindings_xbox(controller, is_driver)
            else:
                self.configure_button_bindings_joystick(controller, is_driver)

        # Configure the additional autos that do not come from pathplanner
        self.configure_additional_autos()

        # Speed limiter useful during initial development
        self._limit_chooser = None
        self.configure_speed_limiter()

        ########################################################
        # Initialize the Smart dashboard for each subsystem
        # Dashboard setup
        self.initialize_dashboard()  # TODO: Deprecate this

        for subsystem in self.subsystems:
            if hasattr(subsystem, "dashboard_initialize") and callable(getattr(subsystem,
                                                                               "dashboard_initialize")):
                subsystem.dashboard_initialize()

        # Configure default command for driving using joystick sticks
        field_relative = self.robot_drive._field_relative

        # MacOS fixup
        right_axis_x = XboxController.Axis.kRightX

        if platform.system().lower() == "darwin":
            hid_axis = self.driver_controller.getHID().Axis
            if hid_axis.kRightX != 2:
                right_axis_x = XboxController.Axis.kLeftTrigger

        # drive_cmd = HolonomicDrive(self,
        #                            self.robot_drive,
        #                            forwardSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftY),
        #                            leftSpeed=lambda: -self.driver_controller.getRawAxis(XboxController.Axis.kLeftX),
        #                            rotationSpeed=lambda: -self.driver_controller.getRawAxis(right_axis_x),
        #                            deadband=OIConstants.DRIVE_DEADBAND,
        #                            field_relative=field_relative,
        #                            rate_limit=True,
        #                            square=True)
        #
        # self.robot_drive.setDefaultCommand(drive_cmd)
        #
        # # TODO: Move pathfinding init here so it is ready for autonomous mode
        #

    @property
    def max_speed(self) -> meters_per_second:
        return self._max_speed * self.robot_drive.drive_scale_factor

    @property
    def max_angular_rate(self) -> radians_per_second:
        return self._max_angular_rate * self.robot_drive.drive_scale_factor

    @property
    def field(self) -> Field2d:
        return self.robot.field

    @property
    def alliance_location(self) -> int:
        """
        Alliance location/position as defined by FMS or chooser.

        Valid values are 1, 2, 3.
        """
        return self._alliance_location

    @property
    def is_red_alliance(self) -> bool:
        """
        Are we in the red alliance?

        The coordinate system is based on the Blue Alliance being to the left (lower x-axis).
        This method provides an 'if' capable function that can be called by routines that need
        a coordinate transformation if we are in the red alliance.
        """
        return self._is_red_alliance

    def check_alliance(self) -> None:
        """
        Support alliance changes up until we start the competition. Default is the blue
        alliance and this function is called during 'disable_periodic' and at the init functions
        for both the Autonomous and Teleop stages.

        Once 'match_started' is True, we are locked into the alliance.
        """
        if not self.robot.match_started:
            # Note that if 'None' is returned for the alliance, we assume Blue
            is_red = DriverStation.getAlliance() == DriverStation.Alliance.kRed
            location = DriverStation.getLocation()

            # Do not change location if not valid
            if location not in (1, 2, 3):
                if location is not None:
                    logger.error(f"Invalid alliance location value: {location}")

                location = self._alliance_location

            if self._is_red_alliance != is_red or self._alliance_location != location:
                # Change of alliance. Update any subsystem or other object that needs
                # to know.
                self._is_red_alliance = is_red
                self._alliance_location = location

                for callback in self._alliance_change_callbacks:
                    callback(is_red, location)

    def register_alliance_change_callback(self, callback: Callable[[bool, int], None]) -> None:
        """
        For subsystems and objects that need to know about alliance changes before the
        match begins.
        """
        self._alliance_change_callbacks.append(callback)

    def set_start_time(self) -> None:  # call in teleopInit and autonomousInit in the robot
        self.start_time = time.time()

    def get_enabled_time(
            self) -> float:  # call when we want to know the start/elapsed time for status and debug messages
        return time.time() - self.start_time

    def elapsed_time(self) -> float:
        return time.time() - self.start_time

    def configure_button_bindings_xbox(self, controller: button.CommandXboxController, is_driver: bool) -> None:
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        if is_driver:
            return self._configure_operator_button_bindings_xbox(controller)

        return self._configure_shooter_button_bindings_xbox(controller)

    def _configure_operator_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    - Robot direction on field. Fwd, Back, Left, Right (from operators perspective)
        RS == Right Stick   - Robot rotation  <- Counter Clockwise  -> Clockwise

        LSB == Left Stick Button
        RSB == Right Stick Button

        D-Pad == Directional Pad

        RB == Right Bumper
        RT == Right Trigger
        LB == Left Bumper
        LT == Left Trigger

        Y == Y Button (Top)
        A == A Button (Bottom)
        X == X Button (Left)
        B == B Button (Right)
        """
        self.robot_drive.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.robot_drive.apply_request(
                lambda: (
                    self.robot_drive.drive_request.with_velocity_x(-self.driver_controller.getLeftY() * self.max_speed)
                    .with_velocity_y(-self.driver_controller.getLeftX() * self.max_speed)
                    .with_rotational_rate(-self.driver_controller.getRightX() * self.max_angular_rate)
                )
            )
        )
        # Idle while the robot is disabled. This ensures the configured
        # neutral mode is applied to the drive motors while disabled.
        idle = swerve.requests.Idle()
        Trigger(DriverStation.isDisabled).whileTrue(
            self.robot_drive.apply_request(lambda: idle).ignoringDisable(True)
        )

        self.driver_controller.a().whileTrue(self.robot_drive.apply_request(lambda: self.robot_drive.brake_request))
        self.driver_controller.b().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.point_at_request.with_module_direction(
                    Rotation2d(-self.driver_controller.getLeftY(), -self.driver_controller.getLeftX())
                )
            )
        )

        self.driver_controller.povUp().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.forward_straight_request.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        self.driver_controller.povDown().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.forward_straight_request.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )

        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (self.driver_controller.back() & self.driver_controller.y()).whileTrue(
            self.robot_drive.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (self.driver_controller.back() & self.driver_controller.x()).whileTrue(
            self.robot_drive.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (self.driver_controller.start() & self.driver_controller.y()).whileTrue(
            self.robot_drive.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (self.driver_controller.start() & self.driver_controller.x()).whileTrue(
            self.robot_drive.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

        # reset the field-centric heading on left bumper press
        self.driver_controller.leftBumper().onTrue(self.robot_drive.runOnce(self.robot_drive.seed_field_centric))

        self.robot_drive.register_telemetry(lambda state: self._logger.telemeterize(state))

        # Robot Driver (primarily responsible for robot path

        controller.a().onTrue(cmd.runOnce(lambda: self.robot_drive.zeroGyro))
        controller.y().whileTrue(cmd.runOnce(lambda: self.robot_drive.lock,
                                             self.robot_drive).repeatedly())

        # TODO:  The Start button is the small menu button with three lines. Do we want this enabled
        #
        # controller.start().onTrue(cmd.runOnce(lambda: self.robot_drive.resetGyroToInitial))

        # controller.leftBumper().onTrue(driveRobotOrientedAngularVelocity)
        # controller.rightBumper().onTrue(driveFieldOrientedAngularVelocity)


    def _configure_shooter_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        pass

    def configure_button_bindings_joystick(self, controller, is_driver: bool) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        logger.debug(f"*** called configureButtonBindings, controller: {controller}, is_driver: {is_driver}")
        pass  # TODO: Not supported at this time. If this is supported, look into places where
        #       XboxController may be used directly (such as in the default drive command
        #       above). Eventually need to abstract this.

    def disablePIDSubsystems(self) -> None:
        """
        Disables all ProfiledPIDSubsystem and PIDSubsystem instances.
        This should be called on robot disable to prevent integral windup.
        """
        self.robot_drive.set_motor_brake(True)

    def get_autonomous_command(self) -> Command:
        """
        :returns: the command to run in autonomous
        """
        command = self._auto_chooser.getSelected()
        return command

    def configure_speed_limiter(self):
        """
        Overall speed limitation scaling factor
        """
        self._limit_chooser = SendableChooser()

        # you can also set the default option, if needed
        self._limit_chooser.setDefaultOption("10%", 0.1)
        self._limit_chooser.addOption("20%", 0.2)
        self._limit_chooser.addOption("40%", 0.4)
        self._limit_chooser.addOption("60%", 0.6)
        self._limit_chooser.addOption("100%", 1.0)

        SmartDashboard.putData("Drive rate limiter", self._limit_chooser)

    def configure_additional_autos(self):
        """
        Add to dashboard "'"Chosen" dialog that allows us to select which 'automation'
        commands to run when we enter the Autonomous phase.

        TODO:  THIS IS JUST A TEST.  USE PATHPLANNER FOR ALL AUTONOMOUS MODE PATHS
        """
        self._auto_chooser.setDefaultOption("Do nothing", self.get_do_nothing())
        self._auto_chooser.addOption("trajectory example", self.get_autonomous_trajectory_example())
        self._auto_chooser.addOption("left blue", self.get_autonomous_left_blue())
        self._auto_chooser.addOption("left red", self.get_autonomous_left_red())

        # If vision based odometry is supported, add in their auto commands
        if self.vision_odometry:
            # TODO: would be a good thing to add into kwargs passed in or do something with
            #       the subsystems class we want to derive
            self._auto_chooser.addOption("Approach tag", self.getApproachTagCommand)

        SmartDashboard.putData("Chosen Auto", self._auto_chooser)

    def get_do_nothing(self) -> Command:
        """
        Have robot stop

        Makes a good default autonomous default while robot is still under test
        """
        return InstantCommand(lambda: self.robot_drive.stop())

    def get_autonomous_left_blue(self) -> Command:
        set_start_pose = ResetXY(self.robot_drive, x=0.783, y=6.686, heading=60)
        drive_forward = RunCommand(lambda: self.robot_drive.arcade_drive(dpeed=1.0, rot=0.0), self.robot_drive)
        stop = InstantCommand(lambda: self.robot_drive.stop())

        command = set_start_pose.andThen(drive_forward.withTimeout(1.0)).andThen(stop)
        return command

    def get_autonomous_left_red(self) -> Command:
        setStartPose = ResetXY(self.robot_drive, x=15.777, y=4.431, heading=-120)
        driveForward = RunCommand(lambda: self.robot_drive.arcade_drive(1.0, 0.0), self.robot_drive)
        stop = InstantCommand(lambda: self.robot_drive.stop())

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getApproachTagCommand(self) -> Command:
        # Approach until the tag takes up 8% of the screen, then drive just a little bit more
        # to compress the robot against the wall the tag is on.

        set_start_pose = ResetXY(self.robot_drive, x=8.7, y=7.3, heading=-180)
        trajectory = JerkyTrajectory(self.robot_drive,
                                     endpoint=(2.59, 3.99, 0),
                                     waypoints=[
                                         (0.775, 7.26, 180),
                                         (6.5, 7.26, -178),
                                         (3.6, 7.1, -175),
                                         (2.17, 5.5, -101)
                                     ],
                                     speed=0.2)  # TODO: Increase speed once we know it works as expected

        follow_tag = FollowObject(self.robot_drive, self.front_camera,
                                  stepSeconds=0.33,
                                  stopWhen=StopWhen(maxSize=8.0),
                                  speed=0.2)  # TODO: Increase speed once we know it works as expected

        drive_forward = ArcadeDrive(self.robot_drive, drive_speed=0.15, rotation_speed=0.0).withTimeout(0.3)  # Keep speed low here..

        return set_start_pose.andThen(trajectory).andThen(follow_tag).andThen(drive_forward)

    def get_autonomous_trajectory_example(self) -> Command:
        command = SwerveTrajectory(self.robot_drive,
                                   speed=+1.0,
                                   waypoints=[
                                       (1.0, 4.0, 0.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                                       (2.5, 5.0, 0.0),  # next waypoint: x=2.5, y=5.0
                                       (3.0, 6.5, 0.0),  # next waypoint
                                       (6.5, 5.0, -90),  # next waypoint
                                   ],
                                   endpoint=(6.0, 4.0, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
                                   flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
                                   stopAtEnd=True)  # to keep driving onto next command, set =False
        return command

    def getTestCommand(self) -> Optional[Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """
        from lib_6107.commands.drivetrain.aimtodirection import AimToDirection

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turn_right = AimToDirection(self.robot_drive, heading=Rotation2d.fromDegrees(-45), speed=0.25)
        turn_left = AimToDirection(self.robot_drive, heading=Rotation2d.fromDegrees(45), speed=0.25)
        back_to_zero = AimToDirection(self.robot_drive, heading=Rotation2d.fromDegrees(0), speed=0.0)

        command = turn_right.andThen(turn_left).andThen(back_to_zero)
        return command

    def initialize_dashboard(self):
        logger.debug("*** called initialize_dashboard")


