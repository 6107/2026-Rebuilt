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
# Adapted from Gene Panov's (Team 714) CommandRevSwerve project (and FRC Python videos)

from typing import Optional

from pathplannerlib.auto import NamedCommands
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import degrees, meters

from lib_6107.commands.command import BaseCommand
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem


class ResetXY(BaseCommand):
    """
    Reset the X & Y position as well as the heading of the robot to a specific value

    Good at startup of autonomous period or during testing of the robot

    TODO: Support field limits and calculate in robot size
    """
    name = "ResetXY"

    def __init__(self, drivetrain: 'DriveSubsystem',
                 x: Optional[meters] = 0.0,
                 y: Optional[meters] = 0.0,
                 heading: Optional[degrees] = 0.0):
        """
        Reset the starting (X, Y) and heading (in degrees) of the robot to where they should be.

        :param drivetrain: drivetrain on which the (X, Y, heading) should be set
        :param x: X
        :param y: X
        :param heading: heading (for example: 0 = "North" of the field, 180 = "South" of the field)
        """
        super().__init__(drivetrain)

        self.position = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(heading))

    @staticmethod
    def pathplanner_register(drivetrain: DriveSubsystem) -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """

        def command(**kwargs) -> ResetXY:
            return ResetXY(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(command().name, command())

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time
        """
        super().initialize()

        self._drivetrain.pose = self.position

    def execute(self):
        """
        nothing to do here, this is an instant command
        """

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return True  # this is an instant command, it finishes right after it initialized

    def end(self, interrupted: bool) -> None:
        """
        nothing to do here, this is an instant command
        """
        super().end(interrupted)
