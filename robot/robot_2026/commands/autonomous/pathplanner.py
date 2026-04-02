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
import os
from typing import Optional

from commands2 import cmd, Command, CommandScheduler
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.events import EventTrigger
from pathplannerlib.logging import PathPlannerLogging
from wpilib import DriverStation, getDeployDirectory
from wpimath.kinematics import ChassisSpeeds

from lib_6107.commands.camera.approach_tag import ApproachTag
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveMove, SwerveToPoint
from lib_6107.pykit.logger import Logger
from lib_6107.pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from robot_2026.commands.autonomous.auto_climber_command import AutoClimberSequence
from robot_2026.commands.climber.climber_commands import ExtendClimber
from robot_2026.commands.climber.climber_commands import RetractClimber
from robot_2026.commands.intake.intake_commands import IntakeCollectFuel
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem

logger = logging.getLogger(__name__)


def configure_auto_builder(drivetrain: DriveSubsystem, container: 'RobotContainer',
                           default_command: Optional[str] = "") -> Optional[LoggedDashboardChooser]:

    # Register named commands first
    register_commands_and_triggers(drivetrain, container)

    # Does pathplanner exist yet?
    file_path = os.path.join(getDeployDirectory(), 'pathplanner', 'settings.json')

    if os.path.isfile(file_path) and os.access(file_path, os.R_OK):
        config = RobotConfig.fromGUISettings()

        AutoBuilder.configure(lambda: drivetrain.get_state().pose,  # Supplier of current robot pose
                              drivetrain.reset_pose,  # Consumer for seeding pose against auto
                              lambda: drivetrain.get_state().speeds,  # Supplier of current robot speeds

                              # Consumer of ChassisSpeeds and feedforwards to drive the robot
                              # TODO:  Create a 'drive-with-path-planned' and set it to following
                              #        see 'drivePathPlanned' in westwood project. Also it calls
                              #        and does a log for each time called'.
                              lambda speeds, feedforwards: drivetrain.set_control(
                                  drivetrain.apply_robot_speeds
                                  .with_speeds(ChassisSpeeds.discretize(speeds, container.robot.getPeriod()))
                                  .with_wheel_force_feedforwards_x(feedforwards.robotRelativeForcesXNewtons)
                                  .with_wheel_force_feedforwards_y(feedforwards.robotRelativeForcesYNewtons)
                              ),
                              PPHolonomicDriveController(
                                  # PID constants for translation
                                  PIDConstants(10.0, 0.0, 0.0),
                                  # PID constants for rotation
                                  PIDConstants(7.0, 0.0, 0.0)
                              ),
                              config,
                              # Assume the path needs to be flipped for Red vs Blue, this is normally the case
                              lambda: (
                                                  DriverStation.getAlliance() or DriverStation.Alliance.kBlue) == DriverStation.Alliance.kRed,
                              drivetrain  # Subsystem for requirements
                              )

        # Setup pykit support in PathPlanner
        command_count: dict[str, int] = {}

        def log_command_function(command: Command, active: bool) -> None:
            name = command.getName()
            count = command_count.get(name, 0) + (1 if active else -1)
            command_count[name] = count
            Logger.recordOutput(f"Commands/{name}", count > 0)

        CommandScheduler.getInstance().onCommandInitialize(lambda c: log_command_function(c, True))
        CommandScheduler.getInstance().onCommandFinish(lambda c: log_command_function(c, False))
        CommandScheduler.getInstance().onCommandInterrupt(lambda c: log_command_function(c, False))

        PathPlannerLogging.setLogCurrentPoseCallback(lambda pose:
                                                     Logger.recordOutput("PathPlanner/CurrentPose", pose))
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose:
                                                    Logger.recordOutput("PathPlanner/TargetPose", pose))
        PathPlannerLogging.setLogActivePathCallback(lambda poses:
                                                    Logger.recordOutput("PathPlanner/CurrentPath", poses))

        return build_auto_chooser(default_command)

    logger.error(f"PathPlanner settings {file_path} not found or is not readable")
    logger.error("Assuming this is an initial run to import Named Commands before creating first Paths/Autos")

    return LoggedDashboardChooser("Autonomous")


def build_auto_chooser(default_auto_name: str = "") -> LoggedDashboardChooser:
    """
    Create and populate a sendable chooser with all PathPlannerAutos in the project and the default auto name selected.

    :param default_auto_name: the name of the default auto to be selected in the chooser
    :return: a sendable chooser object populated with all of PathPlannerAutos in the project
    """
    if not AutoBuilder.isConfigured():
        raise RuntimeError('AutoBuilder was not configured before attempting to build an auto chooser')

    auto_folder_path = os.path.join(getDeployDirectory(), 'pathplanner', 'autos')
    auto_list = os.listdir(auto_folder_path)

    chooser = LoggedDashboardChooser("Autonomous")

    default_auto_added = False

    for auto in auto_list:
        auto = auto.removesuffix(".auto")
        if auto == default_auto_name:
            default_auto_added = True
            chooser.setDefaultOption(auto, AutoBuilder.buildAuto(auto))
        else:
            chooser.addOption(auto, AutoBuilder.buildAuto(auto))

    return chooser


def register_commands_and_triggers(drivetrain: DriveSubsystem, container: 'RobotContainer') -> None:
    # Register Named Commands.
    #
    #   Format is  <command-object-name>, <first-required-parameter>
    commands = [
        # DriveTrian
       (ArcadeDrive,       drivetrain),
       (AimToDirection,    drivetrain),
       (GoToPoint,         drivetrain),
       (SwerveToPoint,     drivetrain),
       (SwerveMove,        drivetrain),

        # Intake
       (IntakeCollectFuel, container),

        # Shooter and associated Feeder

        # Climber
        (RetractClimber, container),
        (ExtendClimber, container),
        (AutoClimberSequence, container),

        # Entertainment
    ]
    for obj, param in commands:
        obj.pathplanner_register(param)

    # And a few special ones depending upon support
    front_camera = drivetrain.container.camera("front")

    if front_camera is not None:
        # TODO: Add more to this location
        ApproachTag.pathplanner_register(drivetrain)

    # Now all the triggers. This is where we can add custom commands that take arguments.
    EventTrigger("Collect Fuel").whileTrue(cmd.PrintCommand("TODO: running intake")) # IntakeAutoCommand(container)

    # Current, TODO: AimAndShoot just spins a 1/2 180 degrees per second for 2 seconds
    EventTrigger("AimAndShoot").whileTrue(cmd.PrintCommand("TODO: running AimAndShoot"))

    # Current, TODO: POS-1 Clime One just spins a 1/2 180 degrees per second for 2 seconds
    EventTrigger("POS-1 Clime One").whileTrue(cmd.PrintCommand("TODO: running POS-1 Clime One"))
