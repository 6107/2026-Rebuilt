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

from commands2 import cmd
from commands2.sysid import SysIdRoutine
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.events import EventTrigger
from pathplannerlib.logging import PathPlannerLogging
from pykit.logger import Logger
from wpilib import DriverStation, getDeployDirectory, SendableChooser
from wpimath.kinematics import ChassisSpeeds

from lib_6107.commands.camera.approach_tag import ApproachTag
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveMove, SwerveToPoint
from robot_2026.commands.intake.intake_commands import IntakeCollectFuel
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem

logger = logging.getLogger(__name__)


def configure_auto_builder(drivetrain: DriveSubsystem, container: 'RobotContainer',
                           default_command: Optional[str] = "") -> Optional[SendableChooser]:

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
                                  .with_speeds(ChassisSpeeds.discretize(speeds, 0.020))  # TODO: Pass in period here
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

        # robot_config = RobotConfig.fromGUISettings()
        #
        # # TODO: What do we need here with pathplanner and assuming pykit is also supported. Also if
        # #       we want the PPLTV Controller, should we use the AutoConstants or DriveConstants.
        # # controller = apriltag.k_pathplanner_holonomic_controller
        # controller = PPLTVController(1 / kwargs["pykit"]["Update Frequency"],
        #                              MAX_SPEED)
        # PathPlanner and AdvantageScope integration
        PathPlannerLogging.setLogActivePathCallback(lambda path: Logger.recordOutput("Odometry/Trajectory",
                                                                                     path))
        PathPlannerLogging.setLogTargetPoseCallback(lambda pose: Logger.recordOutput("Odometry/TrajectorySetpoint",
                                                                                     pose))

        # PathPlannerLogging.setLogCurrentPoseCallback(lambda pose: Logger.recordOutput("PathPlanner/CurrentPose",
        #                                                                               pose))
        # PathPlannerLogging.setLogTargetPoseCallback(lambda pose: Logger.recordOutput("PathPlanner/TargetPose",
        #                                                                              pose))
        # PathPlannerLogging.setLogActivePathCallback(lambda poses: Logger.recordOutput("PathPlanner/CurrentPath",
        #                                                                               poses))
        # TODO: Next relies upon PYKIT support
        SysIdRoutine(SysIdRoutine.Config(1, 7, 10,
                                         lambda state: Logger.recordOutput("Drive/SysIdState",
                                                                           state.name), ),
                     SysIdRoutine.Mechanism((lambda volts: drivetrain.runOpenLoop(volts, volts)),
                                            (lambda: None),
                                            drivetrain))  # Register all the library 'named' commands we may wish to use

        # Load in any Autonomous Commands into the chooser

        # AdvantageKit/pykit provide a LoggedDashboardChooser that can be
        # logged.  TODO: Support that and also use it for any other chooser.

        return AutoBuilder.buildAutoChooser(default_command)

    logger.error(f"PathPlanner settings {file_path} not found or is not readable")
    logger.error("Assuming this is an initial run to import Named Commands before creating first Paths/Autos")

    return None

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
