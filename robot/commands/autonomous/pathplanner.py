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
from wpilib import DriverStation, SendableChooser, getDeployDirectory
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import degreesToRadians

from commands.intake.intake_commands import IntakeCollectFuel
from constants import USE_PYKIT
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.approach_tag import ApproachTag
from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveToPoint, SwerveMove
from subsystems.swervedrive.drivesubsystem import DriveSubsystem

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
                              lambda speeds, feedforwards: drivetrain.set_control(
                                  drivetrain.apply_robot_speeds
                                  .with_speeds(ChassisSpeeds.discretize(speeds, 0.020))
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
        if USE_PYKIT:
            # PathPlanner and AdvantageScope integration
            PathPlannerLogging.setLogActivePathCallback(lambda path: Logger.recordOutput("Odometry/Trajectory",
                                                                                         path))
            PathPlannerLogging.setLogTargetPoseCallback(lambda pose: Logger.recordOutput("Odometry/TrajectorySetpoint",
                                                                                         pose))
            # TODO: Next relies upon PYKIT support
            SysIdRoutine(SysIdRoutine.Config(1, 7, 10,
                                             lambda state: Logger.recordOutput("Drive/SysIdState",
                                                                               state.name),),
                                      SysIdRoutine.Mechanism((lambda volts: drivetrain.runOpenLoop(volts, volts)),
                                                             (lambda: None), drivetrain))    # Register all the library 'named' commands we may wish to use

        # Load in any Autonomous Commands into the chooser
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

    # And a few special ones
    if drivetrain.front_camera is not None:
        ApproachTag.pathplanner_register(drivetrain)

    # Now all the triggers. This is where we can add custom commands that take arguments.

    EventTrigger("Collect Fuel").whileTrue(cmd.PrintCommand("running intake")) # IntakeAutoCommand(container)

    # Current, AimAndShoot just spins a 1/2 180 degrees per second for 2 seconds
    EventTrigger("AimAndShoot").whileTrue(ArcadeDrive(drivetrain,
                                                      rotation_speed=degreesToRadians(180)).withTimeout(2.0))

    # Current, POS-1 Clime One just spins a 1/2 180 degrees per second for 2 seconds
    EventTrigger("POS-1 Clime One").whileTrue(ArcadeDrive(drivetrain,
                                                          rotation_speed=degreesToRadians(180)).withTimeout(2.0))
