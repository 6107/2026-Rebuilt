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
from lib_6107.commands.pathplanner import PathPlanner
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.arcade_drive import ArcadeDrive
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveMove, SwerveToPoint
from lib_6107.commands.vision.approach_tag import ApproachTag
from lib_6107.pykit.logger import Logger
from lib_6107.pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from pathplannerlib.auto import AutoBuilder
from pathplannerlib.auto import RobotConfig
from pathplannerlib.controller import PIDConstants, PPHolonomicDriveController
from pathplannerlib.events import EventTrigger
from pathplannerlib.logging import PathPlannerLogging
from robot_2026.commands.autonomous.auto_climber_command import AutoClimberSequence
from robot_2026.commands.climber.climber_commands import ExtendClimber
from robot_2026.commands.climber.climber_commands import RetractClimber
from robot_2026.commands.intake.intake_commands import IntakeCollectFuel
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem
from wpilib import DriverStation, getDeployDirectory
from wpimath.kinematics import ChassisSpeeds

logger = logging.getLogger(__name__)


class MyPathPlanner(PathPlanner):

    def register_commands_and_triggers(self, drivetrain: DriveSubsystem, container: 'RobotContainer') -> None:
        # Register Named Commands.

        super().register_commands_and_triggers(drivetrain, container)
        #
        #   Format is  <command-object-name>, <first-required-parameter>
        commands = [
            # DriveTrian

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

        # Now all the triggers. This is where we can add custom commands that take arguments.
        EventTrigger("Collect Fuel").whileTrue(cmd.PrintCommand("TODO: running intake")) # IntakeAutoCommand(container)

        # Current, TODO: AimAndShoot just spins a 1/2 180 degrees per second for 2 seconds
        EventTrigger("AimAndShoot").whileTrue(cmd.PrintCommand("TODO: running AimAndShoot"))

        # Current, TODO: POS-1 Clime One just spins a 1/2 180 degrees per second for 2 seconds
        EventTrigger("POS-1 Clime One").whileTrue(cmd.PrintCommand("TODO: running POS-1 Clime One"))
