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
from commands2.button import Trigger
from commands2 import Subsystem, Command, RunCommand, InstantCommand, cmd, button
from phoenix6 import swerve
from wpilib import RobotBase, XboxController, SmartDashboard, SendableChooser, Field2d, DriverStation
from wpimath.units import rotationsToRadians

import constants
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

        # Phoenix6 max settings and telemetry support
        self._max_speed = (1.0 * TunerConstants.speed_at_12_volts)  # speed_at_12_volts desired top speed
        self._max_angular_rate = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity
        self._logger = Telemetry(self._max_speed)

        # Alliance support
        self._is_red_alliance: bool = False  # Coordinate system based off of blue being to the 'left'
        self._alliance_location: int = 1  # Valid numbers are 1, 2, 3
        self._alliance_change_callbacks: List[Callable[[bool, int], None]] = []

        # The driver's controller
        self.driver_controller = button.CommandXboxController(constants.DRIVER_CONTROLLER_PORT)

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
            if isinstance(controller, button.CommandXboxController):
                self.configureButtonBindings_xbox(controller, is_driver)
            else:
                self.configureButtonBindings_Joystick(controller, is_driver)

        # Speed limiter useful during initial development
        self.configureSpeedLimiter()

        # Configure the autos
        self.configureAutos()

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
    def elevator(self) -> Subsystem:
        return self._elevator

    @property
    def intake_left(self) -> Subsystem:
        return self._intake_left

    @property
    def intake_right(self) -> Subsystem:
        return self._intake_right

    @property
    def intake_extend(self) -> Subsystem:
        return self._intake_extend

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

    def configureButtonBindings_xbox(self, controller, is_driver: bool) -> None:

        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        self.robot_drive.setDefaultCommand(
            # Drivetrain will execute this command periodically
            self.robot_drive.apply_request(
                lambda: (
                    self.robot_drive.drive_request.with_velocity_x(-self.driver_controller.getLeftY() * self._max_speed)
                    .with_velocity_y(-self.driver_controller.getLeftX() * self._max_speed)
                    .with_rotational_rate(-self.driver_controller.getRightX() * self._max_angular_rate)
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
                lambda: self.robot_drive.point_request.with_module_direction(
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
        self.driver_controller.leftBumper().onTrue(
            self.robot_drive.runOnce(self.robot_drive.seed_field_centric)
        )

        self.robot_drive.register_telemetry(
            lambda state: self._logger.telemeterize(state)
        )

        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """
        logger.debug(f"*** called configureButtonBindings, controller: {controller}, is_driver: {is_driver}")

        # TODO: call subsystems to do this
        # TODO: The java application had a different commands tied to default and the left/right
        #       bumper (buttons) on the XBox.
        # TODO: Need to reconcile with java cade
        # # driveFieldOrientedDirectAngle      = robot_drive.driveFieldOriented(self.driveDirectAngle)
        # driveFieldOrientedAngularVelocity = self.robot_drive.driveFieldOriented(self.driveAngularVelocity)
        # driveRobotOrientedAngularVelocity = self.robot_drive.driveFieldOriented(self.driveRobotOriented)
        #
        # self.robot_drive.setDefaultCommand(driveFieldOrientedAnglularVelocity)
        if is_driver:
            # Robot Driver (primarily responsible for robot path

            controller.a().onTrue(cmd.runOnce(lambda: self.robot_drive.zeroGyro))
            controller.y().whileTrue(cmd.runOnce(lambda: self.robot_drive.lock,
                                                 self.robot_drive).repeatedly())

            # TODO:  The Start button is the small menu button with three lines. Do we want this enabled
            #
            # controller.start().onTrue(cmd.runOnce(lambda: self.robot_drive.resetGyroToInitial))

            # controller.leftBumper().onTrue(driveRobotOrientedAngularVelocity)
            # controller.rightBumper().onTrue(driveFieldOrientedAngularVelocity)
        else:
            # Robot Operator (responsible for intakes, shooters, ...)
            pass

    def configureButtonBindings_Joystick(self, controller, is_driver: bool) -> None:
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

    def getAutonomousCommand(self) -> Command:
        """
        :returns: the command to run in autonomous
        """
        command = self.chosenAuto.getSelected()
        return command()

    def configureSpeedLimiter(self):
        """
        Overall speed limitation scaling factor
        """
        self.chosenLimiter = SendableChooser()

        # you can also set the default option, if needed
        self.chosenLimiter.addOption("10%", 0.1)
        self.chosenLimiter.addOption("20%", 0.2)
        self.chosenLimiter.addOption("40%", 0.4)
        self.chosenLimiter.addOption("60%", 0.6)
        self.chosenLimiter.setDefaultOption("100%", 1.0)

        SmartDashboard.putData("Drive rate limiter", self.chosenLimiter)

    def configureAutos(self):
        """
        Implement a dashboard "'"Chosen" dialog that allows us to select which 'automation'
        commands to run when we enter the Autonomous phase.
        """
        self.chosenAuto = SendableChooser()

        # TODO: Create more than one for both red and blue aliances, perhaps even have them
        #       mapped out to show our allies so we can maximize the chance of scoring more
        #       during the autonomous stage
        # you can also set the default option, if needed
        self.chosenAuto.setDefaultOption("Do nothing", self.getDoNothing)
        self.chosenAuto.addOption("trajectory example", self.getAutonomousTrajectoryExample)
        self.chosenAuto.addOption("left blue", self.getAutonomousLeftBlue)
        self.chosenAuto.addOption("left red", self.getAutonomousLeftRed)

        # If vision based odometry is supported, add in their auto commands
        if self.vision_odometry:
            # TODO: would be a good thing to add into kwargs passed in or do something with
            #       the subsystems class we want to derive
            self.chosenAuto.addOption("Approach tag", self.getApproachTagCommand)

        SmartDashboard.putData("Chosen Auto", self.chosenAuto)

    def getDoNothing(self) -> Command:
        """
        Have robot stop

        Makes a good default autonomous default while robot is still under test
        """
        return InstantCommand(lambda: self.robot_drive.stop())

    def getAutonomousLeftBlue(self) -> Command:
        setStartPose = ResetXY(x=0.783, y=6.686, heading_degrees=+60, drivetrain=self.robot_drive)
        driveForward = RunCommand(lambda: self.robot_drive.arcade_drive(xSpeed=1.0, rot=0.0), self.robot_drive)
        stop = InstantCommand(lambda: self.robot_drive.stop())

        command = setStartPose.andThen(driveForward.withTimeout(1.0)).andThen(stop)
        return command

    def getAutonomousLeftRed(self) -> Command:
        setStartPose = ResetXY(x=15.777, y=4.431, heading_degrees=-120, drivetrain=self.robot_drive)
        driveForward = RunCommand(lambda: self.robot_drive.arcade_drive(xSpeed=1.0, rot=0.0), self.robot_drive)
        stop = InstantCommand(lambda: self.robot_drive.stop())

        command = setStartPose.andThen(driveForward.withTimeout(2.0)).andThen(stop)
        return command

    def getApproachTagCommand(self) -> Command:
        # Approach until the tag takes up 8% of the screen, then drive just a little bit more
        # to compress the robot against the wall the tag is on.

        set_start_pose = ResetXY(x=8.7, y=7.3, heading_degrees=-180, drivetrain=self.robot_drive)
        trajectory = JerkyTrajectory(drivetrain=self.robot_drive,
                                     endpoint=(2.59, 3.99, 0),
                                     waypoints=[
                                         (0.775, 7.26, 180),
                                         (6.5, 7.26, -178),
                                         (3.6, 7.1, -175),
                                         (2.17, 5.5, -101)
                                     ],
                                     speed=0.2)  # TODO: Increase speed once we know it works as expected

        follow_tag = FollowObject(self.front_camera, self.robot_drive,
                                  stepSeconds=0.33,
                                  stopWhen=StopWhen(maxSize=8.0),
                                  speed=0.2)  # TODO: Increase speed once we know it works as expected

        drive_forward = ArcadeDrive(driveSpeed=0.15, rotationSpeed=0.0,
                                    drivetrain=self.robot_drive).withTimeout(0.3)  # Keep speed low here..

        # Drop corral command (not yet coded)
        drop_corral = None

        return set_start_pose.andThen(trajectory).andThen(follow_tag).andThen(drive_forward).andThen(drop_corral)

    def getAutonomousTrajectoryExample(self) -> Command:
        command = SwerveTrajectory(
            drivetrain=self.robot_drive,
            speed=+1.0,
            waypoints=[
                (1.0, 4.0, 0.0),  # start at x=1.0, y=4.0, heading=0 degrees (North)
                (2.5, 5.0, 0.0),  # next waypoint: x=2.5, y=5.0
                (3.0, 6.5, 0.0),  # next waypoint
                (6.5, 5.0, -90),  # next waypoint
            ],
            endpoint=(6.0, 4.0, -180),  # end point: x=6.0, y=4.0, heading=180 degrees (South)
            flipIfRed=False,  # if you want the trajectory to flip when team is red, set =True
            stopAtEnd=True  # to keep driving onto next command, set =False
        )
        return command

    def getTestCommand(self) -> Optional[Command]:
        """
        :returns: the command to run in test mode ("test dance") to exercise all subsystems
        """

        # example commands that test drivetrain's motors and gyro (our only subsystem)
        turnRight = AimToDirection(degrees=-45, drivetrain=self.robot_drive, speed=0.25)
        turnLeft = AimToDirection(degrees=45, drivetrain=self.robot_drive, speed=0.25)
        backToZero = AimToDirection(degrees=0, drivetrain=self.robot_drive, speed=0.0)

        command = turnRight.andThen(turnLeft).andThen(backToZero)
        return command

    def initialize_dashboard(self):
        logger.info("*** called initialize_dashboard")
        #
        #  Taken from the FRC2429_2025 project   TODO: what do we need here
        #

        #
        # [self.led_indicator_chooser.addOption(key, value) for key, value in self.led.indicators_dict.items()]  # add all the indicators
        # self.led_indicator_chooser.onChange(listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
        #     SetLEDs(container=self, led=self.led, indicator=selected_value)))
        # SmartDashboard.putData('LED Indicator', self.led_indicator_chooser)

        # # commands for pyqt dashboard - please do not remove
        # SmartDashboard.putData('MoveElevatorTop', MoveElevator(container=self, elevator=self.elevator, mode='specified', height=constants.ElevatorConstants.k_max_height-0.005 ))
        # SmartDashboard.putData('MoveElevatorUp', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=0.1 ))
        # SmartDashboard.putData('MoveElevatorDown', MoveElevator(container=self, elevator=self.elevator, mode='incremental', height=-0.1))
        # SmartDashboard.putData('MovePivotUp', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=10))
        # SmartDashboard.putData('MovePivotDown', MovePivot(container=self, pivot=self.pivot, mode='incremental', angle=-10))
        # SmartDashboard.putData('MoveWristUp', MoveWrist(container=self, incremental=True, radians=degreesToRadians(30), timeout=0.2))
        # SmartDashboard.putData('MoveWristDown', MoveWrist(container=self, incremental=True, radians=degreesToRadians(-30), timeout=0.2))
        # SmartDashboard.putData('IntakeOn', RunIntake(container=self, intake=self.intake, value=6, stop_on_end=False))
        # SmartDashboard.putData('IntakeOff', RunIntake(container=self, intake=self.intake, value=0, stop_on_end=False))
        # SmartDashboard.putData('IntakeReverse', RunIntake(container=self, intake=self.intake, value=-6, stop_on_end=False))
        # SmartDashboard.putData('Move climber up', MoveClimber(self, self.climber, 'incremental', math.radians(10)))
        # SmartDashboard.putData('Move climber down', MoveClimber(self, self.climber, 'incremental', math.radians(-10)))
        # SmartDashboard.putData('CANStatus', CANStatus(container=self))
        # SmartDashboard.putData("ResetFlex", Reflash(container=self))
        # SmartDashboard.putData('GoToScore', Score(container=self))
        # SmartDashboard.putData('GoToStow', GoToStow(container=self))
        # SmartDashboard.putData('GoToL1', InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L1)).ignoringDisable(True).andThen(GoToReefPosition(self, 1)))
        # SmartDashboard.putData('GoToL2', InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L2)).ignoringDisable(True).andThen(GoToReefPosition(self, 2)))
        # SmartDashboard.putData('GoToL3', InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L3)).ignoringDisable(True).andThen(GoToReefPosition(self, 3)))
        # SmartDashboard.putData('GoToL4', InstantCommand(lambda: self.robot_state.set_target(RobotState.Target.L4)).ignoringDisable(True).andThen(GoToReefPosition(self, 4)))
        # SmartDashboard.putData('Set valid tag IDs', SetValidTags(self, constants.VisionConstants.k_valid_tags))
        # SmartDashboard.putData('CalElevatorUp', InstantCommand(lambda: self.elevator.offset_encoder_position_meters(0.025)).ignoringDisable(True))
        # SmartDashboard.putData('CalElevatorDown', InstantCommand(lambda: self.elevator.offset_encoder_position_meters(-0.025)).ignoringDisable(True))
        # SmartDashboard.putData('RecalWrist', RecalibrateWrist(container=self).withTimeout(10))
        # SmartDashboard.putData('CalWristUp', InstantCommand(lambda: self.wrist.offset_encoder_position_degrees(2)).ignoringDisable(True))
        # SmartDashboard.putData('CalWristDown', InstantCommand(lambda: self.wrist.offset_encoder_position_degrees(-2)).ignoringDisable(True))
        # # end pyqt dashboard section

        # # quick way to test all scoring positions from dashboard
        # self.score_test_chooser = SendableChooser()
        # [self.score_test_chooser.addOption(key, value) for key, value in self.robot_state.targets_dict.items()]  # add all the indicators
        # self.score_test_chooser.onChange(
        #     listener=lambda selected_value: commands2.CommandScheduler.getInstance().schedule(
        #         cmd.runOnce(lambda: self.robot_state.set_target(target=selected_value))))
        # SmartDashboard.putData('RobotScoringMode', self.score_test_chooser)
        #
        # self.auto_chooser = AutoBuilder.buildAutoChooser('')  # this loops through the path planner deploy directory
        # self.auto_chooser.setDefaultOption('1:  Wait *CODE*', PrintCommand("** Running wait auto **").andThen(commands2.WaitCommand(15)))
        # self.auto_chooser.addOption('2a: Drive 2s Straight *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2)))
        # self.auto_chooser.addOption('2b: Drive 2s To Driver Station *CODE*', PrintCommand("** Running drive by velocity swerve leave auto **").andThen(DriveByVelocitySwerve(self, self.swerve, Pose2d(0.1, 0, 0), 2.5, field_relative=True)))
        # self.auto_chooser.addOption('3a: 1+2 Right from Center *CODE*', OnePlusTwoRight(self, start='center'))  # the working auto that score three coral on right
        # self.auto_chooser.addOption('3b: 1+2 Right from Right *CODE*', OnePlusTwoRight(self, start='right'))  # the working auto that score three coral on right
        # self.auto_chooser.addOption('4a: 1+2 Left from Center *CODE*', OnePlusTwoLeft(self, start='center'))  # simulated left version of code
        # self.auto_chooser.addOption('4b: 1+2 Left from Left *CODE*', OnePlusTwoLeft(self, start='left'))  # simulated left version of code
        # self.auto_chooser.addOption('5:  1+1 Left? *CODE*', OnePlusOne(self))  #  is there any reason for this?
        # self.auto_chooser.addOption('6:  1+0 l4 auto aim', L4PreloadAutoAim(self))
        # # self.auto_chooser.addOption('7: madtwon', MadtownGlaze(self))
        # # self.auto_chooser.addOption('1+2 trough', OnePlusTwoTrough(self))  # not a real auto
        # SmartDashboard.putData('autonomous routines', self.auto_chooser)  #
