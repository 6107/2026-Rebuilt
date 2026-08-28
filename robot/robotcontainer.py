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
from typing import Any, Dict, List,Tuple, Optional

from commands2 import Command, InstantCommand, PrintCommand, RunCommand, Subsystem
from commands2.button import CommandXboxController, Trigger
from commands2.sysid import SysIdRoutine
from phoenix6 import swerve
from wpilib import DriverStation, SmartDashboard, \
    XboxController
from wpimath.geometry import Pose2d, Rotation2d

from constants import DeviceID, FRONT_CAMERA_INFO, LEFT_CAMERA_INFO, REAR_CAMERA_INFO, RIGHT_CAMERA_INFO

from robot_2026.commands.climber.climber_commands import ExtendClimber, RetractClimber, TweekDownClimber, TweekUpClimber
from robot_2026.field.field_2026 import RebuiltField as Field
from robot_2026.generated.tuner_constants import TunerConstants
# from robot_2026.subsystems.ctre_indexer import RevIntakeIndexer as IntakeIndexer
from robot_2026.subsystems.rev_climber import RevClimber as Climber
from robot_2026.subsystems.rev_flywheel import RevFlywheel as Shooter
from robot_2026.subsystems.ctre_pivot import CtreIntakePivot as IntakePivot
from robot_2026.subsystems.rev_roller import RevIntakeRoller as IntakeRoller
from robot_2026.subsystems.simulation.robot_mech import RobotMech
from robot_2026.util.alerts import MyRobotAlerts

from lib_6107.robotcontainer import RobotContainer
from lib_6107.commands.vision.track_tag_command import TrackTagCommand
from lib_6107.pykit.logger import Logger
from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem
from lib_6107.util.numerical_chooser import IntegerEditBox

logger = logging.getLogger(__name__)


class MyRobotContainer(RobotContainer):
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """
    def __init__(self, robot: 'MyRobot') -> None:
        # The robot's subsystems

        ##########################################
        #  Subsystems (fully initialized in base class when it calls into
        #  the subsystem_init() function.  First declare everything that we will create
        #  later in the subsystem_init() function
        #
        self._field: Field = None

        self.robot_drive = None
        self.intake_pivot: IntakePivot | None = None
        self.intake_roller: IntakeRoller | None = None
        # self.indexer: IntakeIndexer | None = None
        self.indexer: None = None
        self.flywheel: Shooter | None = None
        self.climber: Climber | None = None

        self._shooter_rpm_chooser: IntegerEditBox | None = None
        self._intake_rpm_chooser: IntegerEditBox | None = None
        self._indexer_rpm_chooser: IntegerEditBox | None = None

        # Now let the base class handle most of the rest of the work
        super().__init__(robot)

        ##########################################
        #   ALERTS, overwrite it with ours (which are derived from the base alerts
        #
        self._alerts: MyRobotAlerts = MyRobotAlerts(self)

        ########################################################
        # Configure the button bindings
        for controller, port_id in self._controllers:
            if isinstance(controller, CommandXboxController):
                self.configure_button_bindings_xbox(controller, port_id)

            elif controller is not None:
                logger.error(f"Unsupported controller type {type(controller)}")

        # Configure the additional autos that do not come from pathplanner
        self.configure_additional_autos()

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
        #                            square=True)
        #
        # self.robot_drive.setDefaultCommand(drive_cmd)

    @staticmethod
    def create(robot: 'Robot') -> RobotContainer:
        """
        This is passed into the base 'Robot' class's robotInit to be initialized
        near the end of that initialization section.
        """
        return MyRobotContainer(robot)

    @property
    def field(self) -> Field:
        return self._field

    def subsystem_init(self) -> Tuple[Subsystem, ...]:
        """
        Create all subsystems for this years robot
        """
        self._field: Field = Field()
        subsystems: List[Subsystem] = []

        ##########################################
        #  Drivetrain
        #
        self.robot_drive = TunerConstants.create_drivetrain(self)
        subsystems.append(self.robot_drive)

        ##########################################
        #   VISION
        #
        camera_subsystems = self._init_vision_subsystems()
        subsystems.extend(camera_subsystems)

        ##########################################
        self.intake_pivot: IntakePivot | None = None
        self.intake_roller: IntakeRoller | None = None
        # self.indexer: IntakeIndexer | None = None

        self.indexer = None
        self.flywheel: Shooter | None = None
        self.climber: Climber | None = None

        ##########################################
        # NOTE: Disable subsystems that will not be in the next competition
        ##########################################
        #   INTAKE (Pivot & Rollers)
        #
        # Right Pivot Motor should be Inverted
        self.intake_pivot = IntakePivot(self,
                                        DeviceID.INTAKE_RIGHT_PIVOT_DEVICE_ID,
                                        True)

        ###########################################
        #   Roller / rolly-grabbers
        self.intake_roller = IntakeRoller(self, DeviceID.INTAKE_ROLLER_DEVICE_ID, True)

        # ##########################################
        # #   INDEXER (hardware not ready)
        # self.indexer = IntakeIndexer(self, DeviceID.INTAKE_INDEXER_DEVICE_ID, False)
        # ##########################################
        #   SHOOTER/FLYWHEEL (hardware not ready)
        # self.flywheel: Shooter = Shooter(self, DeviceID.SHOOTER_DEVICE_ID, False)

        ##########################################
        #   CLIMBER
        #
        self.climber = Climber(self, DeviceID.CLIMBER_DEVICE_ID, True)

        # Add subsystems that got initialized
        for sub in (self.intake_pivot, self.intake_roller, self.indexer,
                    self.flywheel, self.climber):

            if sub is not None and sub.is_connected:
                subsystems.append(sub)

            elif sub is not None:
                logging.warning(f"Subsystem {sub} not connected to robot or failed to initialize")

        ##########################################
        # Mechanism simulation (MUST BE THE LAST SUBSYSTEM INITIALIZED)
        if self.simulation:
            self._mechanism_2d = RobotMech(self)
            subsystems.append(self._mechanism_2d)

        if self.flywheel is not None:
            self._shooter_rpm_chooser = IntegerEditBox("Shooter RPM",
                                                       initial_value=0,
                                                       minimum_value=0,
                                                       maximum_value=5676)
            SmartDashboard.putData(self._shooter_rpm_chooser.name, self._shooter_rpm_chooser)

        if self.intake_roller is not None:
            self._intake_rpm_chooser = IntegerEditBox("Intake RPM",
                                                       initial_value=0,
                                                       minimum_value=0,
                                                       maximum_value=5676)
            SmartDashboard.putData(self._intake_rpm_chooser.name, self._intake_rpm_chooser)

        if self.indexer is not None:
            self._indexer_rpm_chooser = IntegerEditBox("Indexer RPM",
                                                       initial_value=0,
                                                       minimum_value=0,
                                                       maximum_value=5676)
            SmartDashboard.putData(self._indexer_rpm_chooser.name, self._indexer_rpm_chooser)

        return tuple(subsystems)

    def _init_vision_subsystems(self) -> Tuple[Subsystem, ...]:
        camera_subsystems = []
        # TODO: Do we need to prioritize the cameras so some cameras get serviced first in a multi-vision robot
        for camera_info in (FRONT_CAMERA_INFO, REAR_CAMERA_INFO, RIGHT_CAMERA_INFO, LEFT_CAMERA_INFO):

            camera_subsystem = VisionSubsystem.create(camera_info, self.robot_drive,
                                                      self._field)
            if camera_subsystem is not None:
                camera_subsystems.append(camera_subsystem)
                self._cameras[camera_info["Label"]] = camera_subsystem

        return tuple(camera_subsystems)

    def _configure_driver_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    - Robot direction on field. Fwd, Back, Left, Right (from operators perspective)
        RS == Right Stick   - Robot rotation  <- Counter-Clockwise  -> Clockwise

        LSB == Left Stick Button  - Nothing
        RSB == Right Stick Button - Nothing

        D-Pad == Directional Pad
                - Up        Extend Climber
                - Right     Tweak Climber Down (small increments)       # TODO: Low Priority
                - Down      Retract Climber
                - Left      Tweak Climber Up (small increments)         # TODO: Low Priority

        LB == Left Bumper   Set the default drive to field-centric
        RB == Right Bumper  Set the default drive to robot-centric

        LT == Left Trigger  - Drive Over Left Bump
        RT == Right Trigger - Drive Over Right Bump

        A == A Button (Bottom) - Brake
        B == B Button (Right)  -
        Y == Y Button (Top)    -                # Reset Telemetry. Facing North,  Or Flush Telemetry
        X == X Button (Left)   -

        Start Button (3 lines) -
        Back Button            - Not used by itself
        """
        x_limiter = self.robot_drive.x_drive_limiter
        y_limiter = self.robot_drive.y_drive_limiter
        turn_limiter = self.robot_drive.turn_limiter

        self.robot_drive.setDefaultCommand(
            # Drivetrain will execute this command periodically. The slew rate limiters are
            # applied to both the drive and turn aspects of the command.
            self.robot_drive.apply_request(
                lambda: (
                    self.robot_drive.drive_request.with_velocity_x(
                        x_limiter.calculate(-self._driver_controller.getLeftY() * self.max_speed))
                    .with_velocity_y(y_limiter.calculate(-self._driver_controller.getLeftX() * self.max_speed))
                    .with_rotational_rate(
                        turn_limiter.calculate(-self._driver_controller.getRightX() * self.max_angular_rate))
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
        if self.camera("front") is not None:
            def turn_to_object():
                x = self.camera("front").x_offset
                print(f"x={x}")
                turn_speed = -0.005 * x
                self.robot_drive.rotate(turn_speed)

            controller.leftTrigger(threshold=0.25).whileTrue(RunCommand(turn_to_object,
                                                                        self.robot_drive))
            controller.leftTrigger(threshold=0.25).onFalse(InstantCommand(lambda: self.robot_drive.stop()))

        # Right Trigger - Follow the best AprilTag around the room

        # Left Bumper - Reset the default drive to field-centric
        controller.leftBumper().onTrue(InstantCommand(lambda: self.robot_drive.set_field_centric_drive(True)))

        # Left Bumper - Reset the default drive to robot-centric
        controller.rightBumper().onTrue(InstantCommand(lambda: self.robot_drive.set_field_centric_drive(False)))

        # A Button - Brake
        controller.a().whileTrue(
            self.robot_drive.apply_request(lambda: self.robot_drive.brake_request)
        )
        # B Button -

        # Y Button - Reset Telemetry
        # controller.y().onTrue(TODO COMMAND(self.robot_drive, x=1.0, y=4.0, heading=0))
        controller.y().onTrue(PrintCommand("Y BUTTON PRESS = TODO"))

        # # POV-UP: Drive forward at 1/2 speed
        # controller.povUp().whileTrue(
        #     self.robot_drive.apply_request(
        #         lambda: self.robot_drive.forward_straight_request.with_velocity_x(0.5).with_velocity_y(0)
        #     )
        # )
        # # POV-DOWN: Drive backwards at 1/2 speed
        # controller.povDown().whileTrue(
        #     self.robot_drive.apply_request(
        #         lambda: self.robot_drive.forward_straight_request.with_velocity_x(-0.5).with_velocity_y(0)
        #     )
        # )
        # # Left Bumper - reset the field-centric heading on left bumper press
        # controller.leftBumper().onTrue(
        #     self.robot_drive.runOnce(self.robot_drive.seed_field_centric)
        # )
        if self.climber is not None and self.climber.is_connected:
            # POV-UP: Retract the climbing arm (robot goes up) - POV-UP is a zero (0) degree reading
            climb_up = controller.povUp()
            climb_up.onTrue(ExtendClimber(self))

            # POV-DOWN: Extend the climbing arm (robot goes down)
            climb_down = controller.povDown()
            climb_down.onTrue(RetractClimber(self))

            # POV-RIGHT  (Climber Down in small increments
            tweak_down = controller.povRight()
            tweak_down.onTrue(TweekDownClimber(self))

            # POV-LEFT  (Climber Up in small increments)
            tweak_up = controller.povLeft()
            tweak_up.onTrue(TweekUpClimber(self))

        # Start Button
        # TODO -> add support : controller.start().onTrue(cmd.runOnce(lambda: self.robot_drive.resetGyroToInitial))

    def _configure_operator_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    -
        RS == Right Stick   -

        LSB == Left Stick Button
        RSB == Right Stick Button

        D-Pad == Directional Pad
                - Up       - Retract the climbing arm (robot goes up)
                - Right    - Lower the Intake (pivot down)
                - Down     - Extend the climbing arm (robot goes down)
                - Left     - Raise the Intake (pivot up)

        LB == Left Bumper
        RB == Right Bumper - Follow an april tag around the room

        LT == Left Trigger  - Keep robot shooter pointing toward our alliance hub (while pressed)
        RT == Right Trigger

        A == A Button (Bottom) - Brake
        B == B Button (Right)  -
        Y == Y Button (Top)    -
        X == X Button (Left)   -

        Start Button (three lines)  - Reset Gyro
        Back Button
        """
        # Right trigger - Start the shooter
        # TODO: Figure out our tolerances
        # TODO: Adjust RPM higher. Start out slow so we cantest it
        if self.flywheel is not None and self.flywheel.is_connected:
            rpm = 120                   # TODO : tie into chooser
            tolerance = 40

            controller.button(XboxController.Axis.kLeftTrigger).onTrue(
                InstantCommand(lambda: self.flywheel.set_velocity_goal(rpm, tolerance))
            ).onFalse(
                InstantCommand(lambda: self.flywheel.stop())
            )

        if self.climber is not None and self.climber.is_connected:
            # POV-UP: Retract the climbing arm (robot goes up) - POV-UP is a zero (0) degree reading
            climb_up = controller.povUp()
            retract_command = RetractClimber(self)
            climb_up.whileTrue(retract_command.andThen(InstantCommand(lambda: self.climber.stop())))

            # POV-DOWN: Extend the climbing arm (robot goes down)
            climb_down = controller.povDown()
            extend_command = ExtendClimber(self)
            climb_down.whileTrue(extend_command.andThen(InstantCommand(lambda: self.climber.stop())))

        if self.intake_pivot is not None and self.intake_pivot.is_connected:
            logger.info("Enabling Driver control of intake pivot. PovLeft is UP, PovRight is Down")
            rotate_down = controller.povLeft()
            up_command = InstantCommand(lambda: self.intake_pivot.pivot_up())
            rotate_down.onTrue(up_command)

            rotate_up = controller.povRight()
            down_command = InstantCommand(lambda: self.intake_pivot.pivot_down())
            rotate_up.onTrue(down_command)

            """
            pov_left_trigger = controller.povLeft()
            pov_right_trigger = controller.povRight()
            x_button_trigger = controller.x()

            #down_trigger = pov_right_trigger.and_(x_button_trigger.negate())
            down_trigger = pov_right_trigger
            down_command = InstantCommand(lambda: self.intake_pivot.pivot_down())
            down_trigger.onTrue(down_command)

            #up_trigger = pov_left_trigger.and_(x_button_trigger.negate())
            up_trigger = pov_left_trigger
            up_command = InstantCommand(lambda: self.intake_pivot.pivot_down())
            up_trigger.onTrue(up_command)

            # # Incremental Adjustments
            # tweak_down_trigger = pov_right_trigger.and_(x_button_trigger)
            # tweak_down_command = InstantCommand(lambda: self.intake_pivot.pivot_tweak_down(1))
            # tweak_down_trigger.whileTrue(tweak_down_command)
            #
            # tweak_up_trigger = pov_left_trigger.and_(x_button_trigger)
            # tweak_up_command = InstantCommand(lambda: self.intake_pivot.pivot_tweak_up(1))
            # tweak_up_trigger.whileTrue(tweak_up_command)
            """

    def _configure_calibration_button_bindings_xbox(self, controller: CommandXboxController) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.

        LS == Left Stick    - Robot direction on field. Fwd, Back, Left, Right (from operators perspective)
        RS == Right Stick   - Robot rotation  <- Counter-Clockwise  -> Clockwise

        LSB == Left Stick Button
        RSB == Right Stick Button

        D-Pad == Directional Pad
                - Up
                - Right    - Rotate 360 degrees clockwise
                - Down
                - Left     - Rotate 360 degrees counter-clockwise

        LB == Left Bumper
        RB == Right Bumper

        LT == Left Trigger
        RT == Right Trigger

        A == A Button (Bottom) - Brake
        B == B Button (Right)  - Align all wheels in direction or Left Stick's Y value
        Y == Y Button (Top)    -
        X == X Button (Left)   -

        Start Button (three lines)  - Reset Gyro
        Back Button
        """
        controller.a().whileTrue(
            self.robot_drive.apply_request(lambda: self.robot_drive.brake_request)
        )
        # B Button - Align all wheels in the direction of the left stick Y value
        controller.b().whileTrue(
            self.robot_drive.apply_request(
                lambda: self.robot_drive.point_at_request.with_module_direction(
                    Rotation2d(-controller.getLeftY(),
                               -controller.getLeftX())
                ).with_drive_request(swerve.SwerveModule.DriveRequestType.VELOCITY)
            )
        )
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
        # reset the field-centric heading on left bumper press
        controller.leftBumper().onTrue(self.robot_drive.runOnce(self.robot_drive.seed_field_centric))

        # Right bumper - track an apriltag around the room
        front_camera: VisionSubsystem | None = self._cameras.get("front")

        if front_camera is not None:
            track_any_tag = TrackTagCommand(self.robot_drive, front_camera, 0)
            right_bumper_pressed = controller.axisGreaterThan(XboxController.Axis.kRightTrigger,
                                                              threshold=0.5)
            right_bumper_pressed.whileTrue(track_any_tag)

    def configure_additional_autos(self):
        """
        Add to dashboard "'"Chosen" dialog that allows us to select which 'automation'
        commands to run when we enter the Autonomous phase.

        TODO:  THIS IS JUST A TEST.  USE PATHPLANNER FOR ALL AUTONOMOUS MODE PATHS
        """

        # Put sys IDs to run as automation (if not in competition)
        # if not DriverStation.isFMSAttached() and not self.simulation:
        #     if self.climber is not None:
        #         self.auto_chooser.addOption("Climber SysID", self.climber.sys_id_routine(self.climber))
        #
        #     if self.intake_pivot is not None:
        #         self.auto_chooser.addOption("Intake SysID", self.intake_pivot.sys_id_routine(self.intake_pivot))

        # Auto-started end-of-autonomous mode command (Climb the ladder 1 - Rung)
        self._auto_end_chooser.setDefaultOption("Do nothing", self.get_do_nothing(stop=False))
        # self._auto_end_chooser.addOption("Do nothing", self.get_do_nothing(stop=False))

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
        # Some year specific work first

        # Update dashboard/pykit data related to match state
        we_won = self._field.won_autonomous
        if we_won is not None:
            Logger.recordOutput("Game/WonAuto", we_won)
            Logger.recordOutput("Game/HubActive", self._field.hub_active)
            Logger.recordOutput("Game/HubAboutToChange", self._field.hub_about_to_change)
            Logger.recordOutput("Game/ShouldGoToHub", self._field.should_go_to_hub)
            Logger.recordOutput("Game/ShouldGoToFeed", self._field.should_go_to_feed)
            Logger.recordOutput("Game/HubDistance", self._field.distance_to_hub)
            # NOTE: The 'flywheel' related values we show are updated in the Flywheel periodic routine

            if self.simulation:
                pose: Pose2d = drive.pose
                SmartDashboard.putBoolean(f"{self._long_name}/InAllianceLeftQuadtrant",
                                          self._field.in_my_alliance_zone(pose.x) is True and
                                          self._field.in_left_zone_area(pose.y) is True)

                SmartDashboard.putBoolean(f"{self._long_name}/InAllianceRightQuadtrant",
                                          self._field.in_my_alliance_zone(pose.x) is True and
                                          self._field.in_right_zone_area(pose.y) is True)

                SmartDashboard.putBoolean(f"{self._long_name}/InNeutralLeftQuadtrant",
                                          self._field.in_neutral_zone(pose.x) is True and
                                          self._field.in_left_zone_area(pose.y) is True)

                SmartDashboard.putBoolean(f"{self._long_name}/InNeutralRightQuadtrant",
                                          self._field.in_neutral_zone(pose.x) is True and
                                          self._field.in_right_zone_area(pose.y) is True)

                SmartDashboard.putBoolean(f"{self._long_name}/InOpponentLeftQuadtrant",
                                          self._field.in_my_opponents_zone(pose.x) is True and
                                          self._field.in_left_zone_area(pose.y) is True)

                SmartDashboard.putBoolean(f"{self._long_name}/InOpponentRightQuadtrant",
                                          self._field.in_my_opponents_zone(pose.x) is True and
                                          self._field.in_right_zone_area(pose.y) is True)

        # Now the base class
        super().robotPeriodic()
