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
import os
import time
from typing import Any, Callable, Dict, List, Optional

from commands2 import button, cmd, Command, InstantCommand, PrintCommand, RunCommand, Subsystem
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from ntcore import NetworkTableInstance
from phoenix6 import swerve
from pykit.logger import Logger
from pykit.alertlogger import AlertLogger
from wpilib import Alert, DriverStation, Field2d, getDeployDirectory, RobotBase, SendableChooser, SmartDashboard, \
    XboxController
from wpimath.geometry import Rotation2d
from wpimath.units import meters, meters_per_second, radians_per_second, rotationsToRadians

import constants
from commands.autonomous import pathplanner
from commands.swervedrive.point_towards_location import PointTowardsLocation
from constants import DeviceID, FRONT_CAMERA_INFO, LEFT_CAMERA_INFO, REAR_CAMERA_INFO, RIGHT_CAMERA_INFO, \
    ROBOT_X_WIDTH_DEFAULT, ROBOT_Y_WIDTH_DEFAULT
from field.field_2026 import RebuiltField as Field
from generated.tuner_constants import TunerConstants
from lib_6107.commands.camera.track_tag_command import TrackTagCommand
from lib_6107.commands.drivetrain.reset_xy import ResetXY
from lib_6107.constants import DEFAULT_ROBOT_FREQUENCY
from lib_6107.subsystems.pykit.robot_state import RobotState
from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem
from subsystems.rev_shooter import RevShooter as Shooter

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

        self.start_time = time.time()
        self.robot = robot
        self.network_table = NetworkTableInstance.getDefault()

        self.simulation = RobotBase.isSimulation()

        # Phoenix6 max settings and telemetry support. During the actual drive command or the
        # arcade_drive function, we apply any scale factor to limit speed
        self._max_speed: meters_per_second = constants.MAX_SPEED  # speed_at_12_volts desired top speed
        self._max_angular_rate: radians_per_second = rotationsToRadians(0.75)  # 3/4 of a rotation per second max angular velocity

        # Alliance support
        self._is_red_alliance: bool = RobotBase.isSimulation()  # Coordinate system based off of blue being to the 'left'
        self._alliance_location: int = 1  # Valid numbers are 1, 2, 3
        self._alliance_change_callbacks: List[Callable[[bool, int], None]] = []
        #
        # TODO: Alliance changes do not seem to work here. Rely on old polling method
        # prefixes = ["FMSInfo/IsRedAlliance", "FMSInfo/IsRedAlliance"]
        # self.alliance_change_listener = NetworkTableListener.createListener(self.network_table,
        #                                                                     prefixes,
        #                                                                     EventFlags.kValueAll,
        #                                                                     self._on_alliance_change)
        # The driver's controller
        self.driver_controller = CommandXboxController(constants.DRIVER_CONTROLLER_PORT)

        # TODO: WPILib has a wpimath.fileter.SlewRateLimiter to make joystick more gentle. Look into this
        #       See https://github.com/robotpy/examples/blob/main/SwerveBot/robot.py for an example

        # Shooter's controller
        self.shooter_controller: XboxController = XboxController(constants.SHOOTER_CONTROLLER_PORT)  # On USB-port

        ########################################################
        # Subsystem initialization
        #
        period = robot.getPeriod() or DEFAULT_ROBOT_FREQUENCY

        ##########################################
        # Subsystem Initialization
        #
        # The robot core code will already call the periodic() function
        # as needed, but having our own list (iterated in order) allows us to move much of
        # the other subsystem 'tasks' into a generic loop.
        self.subsystems: List[Subsystem] = []

        ##########################################
        #  Drivetrain
        #
        # self.robot_drive = DriveSubsystem(self, **drive_kwargs)
        self.robot_drive = TunerConstants.create_drivetrain(self)
        self.subsystems.append(self.robot_drive)

        ##########################################
        #   VISION
        #
        self._cameras: Dict[str, Any] = {}
        # self._localizer: Optional[Subsystem] = None
        self._field: Field = Field()

        camera_subsystems = self._init_vision_subsystems()
        self.subsystems.extend(camera_subsystems)

        ##########################################
        #   SHOOTER
        #
        self.shooter = Shooter(self, DeviceID.SHOOTER_DEVICE_ID, False)

        ##########################################
        #   INDEXER
        #

        ##########################################
        #   INTAKE
        #

        ##########################################
        #   CLIMBER
        #

        ##########################################
        #   ALERTS
        #
        AlertLogger.registerGroup("Alerts")

        self.driverDisconnected = Alert("Driver controller disconnected (port 0)", Alert.AlertType.kWarning)
        self.operatorDisconnected = Alert("Operator controller disconnected (port 1)", Alert.AlertType.kWarning)
        self.deadInTheWaterAlert = Alert("No auto selected!!!", Alert.AlertType.kWarning)

        # TODO: also maybe a vibrate...
        self.shiftActiveAlert = Alert("SHIFT ACTIVE!", Alert.AlertType.kInfo)
        self.shiftActiveAlert.set(True)

        ##########################################
        #   TELEMETRY
        #
        # Disabled since we are using pykit for now. Kept here in case we need it for some
        # tuning with other tools
        self._phoenix_telemetry = None
        # self._phoenix_telemetry = Telemetry(self._max_speed)

        ##########################################
        #   PathPlanner.  Do this last since it may pull in commands that need the previously
        #                 initialized subsystems.
        # Init the Auto chooser.  PathPlanner init will fill in our choices
        self._auto_chooser = pathplanner.configure_auto_builder(self.robot_drive, self, "")
        self._auto_end_chooser = SendableChooser()

        self._robot_x_width: meters = ROBOT_X_WIDTH_DEFAULT
        self._robot_y_width: meters = ROBOT_Y_WIDTH_DEFAULT

        try:
            path = os.path.join(getDeployDirectory(), 'pathplanner', 'settings.json')

            with open(path, 'r') as f:
                settings = json.loads(f.read())
                self._robot_x_width = settings.get("robotWidth", self._robot_x_width)
                self._robot_y_width = settings.get("robotWidth", self._robot_y_width)

        except FileNotFoundError:
            pass

        ########################################################
        # Configure the button bindings
        for controller, is_driver in ((self.driver_controller, True),
                                      (self.shooter_controller, False)):
            if isinstance(controller, CommandXboxController):
                self.configure_button_bindings_xbox(controller, is_driver)
            else:
                logger.error(f"Unsupported controller type {type(controller)}")

        # Configure the additional autos that do not come from pathplanner
        self.configure_additional_autos()

        # Register telemetry support if not running pykit/AdvantageScope
        if self._phoenix_telemetry is not None:
            self.robot_drive.register_telemetry(lambda state: self._phoenix_telemetry.telemeterize(state))

        # Speed limiter useful during initial development
        self._limit_chooser = None
        self.configure_speed_limiter()

        ########################################################
        # Initialize the Smart dashboard for each subsystem
        # Dashboard setup
        for subsystem in self.subsystems:
            if hasattr(subsystem, "dashboard_initialize") and callable(getattr(subsystem,
                                                                               "dashboard_initialize")):
                subsystem.dashboard_initialize()
        #########################################################
        # Specific commands based on time remaining

        self._autonomous_end_game_command = None

        # TODO: Currently we are always field centric wrt commands and using Pathplanner
        # # Configure default command for driving using joystick sticks
        # field_relative = self.robot_drive.field_relative
        #
        # # MacOS fixup
        # right_axis_x = XboxController.Axis.kRightX
        #
        # if platform.system().lower() == "darwin":
        #     hid_axis = self.driver_controller.getHID().Axis
        #     if hid_axis.kRightX != 2:
        #         right_axis_x = XboxController.Axis.kLeftTrigger
        #
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

    @property
    def max_speed(self) -> meters_per_second:
        return self._max_speed * self.robot_drive.drive_scale_factor

    @property
    def max_angular_rate(self) -> radians_per_second:
        return self._max_angular_rate * self.robot_drive.drive_scale_factor

    @property
    def robot_x_width(self) -> meters:
        return self._robot_x_width

    @property
    def robot_y_width(self) -> meters:
        return self._robot_y_width

    @property
    def field(self) -> Field2d:
        return self.robot.field

    def camera(self, label: str) -> Optional[VisionSubsystem]:
        return self._cameras.get(label)

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

    def get_elapsed_time(self) -> float:
        """
        Called when we want to know the start/elapsed time for status and debug messages
        """
        return time.time() - self.start_time

    def elapsed_time(self) -> float:
        return time.time() - self.start_time

    def configure_button_bindings_xbox(self, controller: button.CommandXboxController, is_driver: bool) -> None:
        # Note that X is defined as forward according to WPILib convention,
        # and Y is defined as to the left according to WPILib convention.
        if is_driver:
            return self._configure_driver_button_bindings_xbox(controller)

        return self._configure_shooter_button_bindings_xbox(controller)

    def _configure_driver_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    - Robot direction on field. Fwd, Back, Left, Right (from operators perspective)
        RS == Right Stick   - Robot rotation  <- Counter Clockwise  -> Clockwise

        LSB == Left Stick Button
        RSB == Right Stick Button

        D-Pad == Directional Pad
                - Up        Drive forward in X-Direction at 1/2 speed
                - Right
                - Down      Drive backward in X-Direction at 1/2 speed
                - Left

        LB == Left Bumper    Resets the rotation of the robot pose to the given value from
                             the ForwardPerspectiveValue.OPERATOR_PERSPECTIVE perspective
        RB == Right Bumper

        LT == Left Trigger  - Rotate (in-place) toward best AprilTag. Stop when trigger released.
        RT == Right Trigger - Follow the best AprilTag around the room. Stop when trigger released.

        A == A Button (Bottom) - Brake
        B == B Button (Right)  - Align all wheels in direction of the left-stick Y value
        Y == Y Button (Top)    - Reset Telemetry. Facing North
        X == X Button (Left)   -

        Start Button (three lines)  - Reset Gyro
        Back Button
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
        # Left Trigger - Rotate (in-place) toward best AprilTag.
        def turn_to_object():
            x = self.camera("front").x_offset
            print(f"x={x}")
            turn_speed = -0.005 * x
            self.robot_drive.rotate(turn_speed)

        controller.leftTrigger(threshold=0.25).whileTrue(RunCommand(turn_to_object,
                                                                    self.robot_drive))
        controller.leftTrigger(threshold=0.25).onFalse(InstantCommand(lambda: self.robot_drive.stop()))

        # Right Trigger - Follow the best AprilTag around the room

        # A Button - Brake
        controller.a().whileTrue(
            self.robot_drive.apply_request(lambda: self.robot_drive.brake_request)
        )
        # B Button - Align all wheels in the direction of the left stick Y value
        controller.b().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.point_at_request.with_module_direction(
                    Rotation2d(-controller.getLeftY(),
                               -controller.getLeftX())
                )
            )
        )
        # Y Button - Reset x/y to defaults and heading to 'North' (0 degrees)
        controller.b().whileTrue(
            ResetXY(self.robot_drive, x=1.0, y=4.0, heading=0)
        )
        # POV-UP: Drive forward at 1/2 speed
        controller.povUp().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.forward_straight_request.with_velocity_x(0.5).with_velocity_y(0)
            )
        )
        # POV-DOWN: Drive backwards at 1/2 speed
        controller.povDown().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.forward_straight_request.with_velocity_x(-0.5).with_velocity_y(0)
            )
        )
        # Left Bumper - reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(
            self.robot_drive.runOnce(self.robot_drive.seed_field_centric)
        )
        # Start Button
        controller.start().onTrue(cmd.runOnce(lambda: self.robot_drive.resetGyroToInitial))

        # Back Buttons (complex.  TODO Investigate these)
        # Run SysId routines when holding back/start and X/Y.
        # Note that each routine should be run exactly once in a single log.
        (controller.back() & controller.y()).whileTrue(
            self.robot_drive.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        (controller.back() & controller.x()).whileTrue(
            self.robot_drive.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )
        (controller.start() & controller.y()).whileTrue(
            self.robot_drive.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        (controller.start() & controller.x()).whileTrue(
            self.robot_drive.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )

    def _configure_shooter_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    -
        RS == Right Stick   -

        LSB == Left Stick Button
        RSB == Right Stick Button

        D-Pad == Directional Pad
                - Up
                - Right
                - Down
                - Left

        LB == Left Bumper
        RB == Right Bumper - Follow an april tag around the room

        LT == Left Trigger  - Keep robot shooter pointing toward our alliance hub (while pressed)
        RT == Right Trigger

        A == A Button (Bottom) -
        B == B Button (Right)  -
        Y == Y Button (Top)    -
        X == X Button (Left)   -

        Start Button (three lines)  - Reset Gyro
        Back Button
        """
        # Right trigger - Start the shooter
        # TODO: Figure out our tolerances
        # TODO: Adjust RPM higher. Start out slow so we cantest it
        rpm = 120
        tolerance = 40

        controller.button(XboxController.Axis.kLeftTrigger).onTrue(
            InstantCommand(lambda: self.shooter.set_velocity_goal(rpm, tolerance))
        ).onFalse(
            InstantCommand(lambda: self.shooter.stop())
        )

        track_any_tag = TrackTagCommand(self.robot_drive, self._cameras["front"], 0)
        right_bumper_pressed = controller.axisGreaterThan(XboxController.Axis.kRightTrigger,
                                                           threshold=0.5)
        right_bumper_pressed.whileTrue(track_any_tag)

        # Left trigger - create a command for keeping the robot nose pointed towards the hub
        keep_pointing_towards_hub = PointTowardsLocation(self.robot_drive,
                                                         self._field.hub_location(False),
                                                         self._field.hub_location(True))

        # set up a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%
        when_left_trigger_pressed = controller.axisGreaterThan(XboxController.Axis.kLeftTrigger,
                                                              threshold=0.5)
        # connect the command to its trigger
        when_left_trigger_pressed.whileTrue(keep_pointing_towards_hub)

        # Right bumper - track an apriltag around the room
        track_any_tag = TrackTagCommand(self.robot_drive, self._cameras["front"], 0)
        right_bumper_pressed = controller.axisGreaterThan(XboxController.Axis.kRightTrigger,
                                                           threshold=0.5)
        right_bumper_pressed.whileTrue(track_any_tag)

    def _init_vision_subsystems(self) -> List[Subsystem]:
        camera_subsystems = []

        for camera_info in (FRONT_CAMERA_INFO, REAR_CAMERA_INFO, RIGHT_CAMERA_INFO, LEFT_CAMERA_INFO):

            camera_subsystem, localizer_subsystem = VisionSubsystem.create(camera_info,
                                                                           self._field,
                                                                           self.robot_drive)
            if camera_subsystem is not None:
                camera_subsystems.append(camera_subsystem)
                self._cameras[camera_info["Label"]] = camera_subsystem

            if localizer_subsystem is not None:
                camera_subsystems.append(localizer_subsystem)

        return camera_subsystems

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

    def get_autonomous_end_game_command(self) -> Optional[Command]:
        """
        :returns: the command to run at the end of autonomous
        """
        return self._auto_end_chooser.getSelected()

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
        self._auto_chooser.setDefaultOption("Do nothing", self.get_do_nothing(stop=True))

        # Auto-started end-of-autonomous mode command (Climb the ladder 1 - Rung)
        self._auto_end_chooser.setDefaultOption("Do nothing", self.get_do_nothing(stop=False))

        SmartDashboard.putData("Chosen Auto", self._auto_chooser)
        SmartDashboard.putData("Chosen Auto End Game", self._auto_end_chooser)

    def get_do_nothing(self, stop: Optional[bool] = True) -> Command:
        """
        Have robot stop

        Makes a good default autonomous default while robot is still under test
        """
        if stop:
            return InstantCommand(lambda: self.robot_drive.stop())

        return PrintCommand("Do-Nothing Command")

    def robotPeriodic(self) -> None:
        """
        This is called from Robot.robotPeriodic() after the Phoenix6 signal updates
        are requested.

        Also remember that the SubSystem 'periodic' calls are from the CommandScheduler
        run() method which is AFTER robotPeriodic returns

        This should update the saved robot state that can then be used later by
        any commands.
        """
        # Update all our I/O values. This periodic call will also call the pykit
        # logger.
        drive = self.robot_drive

        RobotState.periodic(drive.pose,
                            Rotation2d(drive.gyro.inputs.yaw),
                            drive.get_robot_3d(drive.pose, Rotation2d(drive.gyro.inputs.yaw)),
                            drive.gyro.inputs.yaw_timestamp,
                            drive.get_angular_velocity(),
                            drive.get_field_relative_speeds(),
                            drive.get_module_positions())
        # TODO: Add more mechanisms as they get coded

        we_won = self._field.won_autonomous
        if we_won is not None:
            Logger.recordOutput("Game/WonAuto", we_won)
            Logger.recordOutput("Game/HubActive", self._field.hub_active)

        self.update_alerts()
        #
        # TODO: Next returns 3d poses for all mechanisms.  Might be good for our devices as well.
        # Logger.recordOutput("Component Poses", RobotMechanism.getPoses())

    def update_alerts(self):
        self.driverDisconnected.set(not DriverStation.isJoystickConnected(0))
        self.operatorDisconnected.set(not DriverStation.isJoystickConnected(1))

        self.deadInTheWaterAlert.set(self._auto_chooser.getSelected() == self.get_do_nothing)
