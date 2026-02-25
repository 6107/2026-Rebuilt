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

from typing import Callable, Optional

from pathplannerlib.auto import NamedCommands
from wpimath.units import meters_per_second, radians_per_second

from lib_6107.commands.command import BaseCommand
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem


class ArcadeDrive(BaseCommand):
    """
    Classic arcade drive forward request as a command.

    Drive the robot at `drive_speed` and `rotation_speed` until this command is terminated.

    Both the drive speed and rotational speed can be constants or a function can be provided
    to retrieve it from elsewhere.
    """
    name = "ArcadeDrive"

    def __init__(self, drivetrain: DriveSubsystem,
                 drive_speed: Optional[meters_per_second | Callable[[], meters_per_second]] = 0.0,
                 rotation_speed: Optional[radians_per_second | Callable[[], radians_per_second]] = 0.0,
                 assume_manual_input: Optional[bool] = False):

        super().__init__(drivetrain)

        self._start_time: float = 0
        self._drive_speed = drive_speed if callable(drive_speed) else lambda: max(0.0, drive_speed)
        self._rotation_speed = rotation_speed if callable(rotation_speed) else lambda: max(0.0, rotation_speed)
        self._assume_manual_input = assume_manual_input

    @staticmethod
    def pathplanner_register(drivetrain: DriveSubsystem) -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> ArcadeDrive:
            return ArcadeDrive(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(command().name, command())

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time
        """
        super().initialize()

    def execute(self) -> None:
        """
        The main body of a command. Called repeatedly while the command is scheduled.
        """
        drive_speed = self._drive_speed()        # get the drive speed from the joystick or wherever it comes from
        rotation_speed = self._rotation_speed()  # get the turn speed from the joystick or wherever it comes from
        self._drivetrain.arcade_drive(drive_speed,
                                      rotation_speed,
                                      assume_manual_input=self._assume_manual_input,
                                      field_relative=True)

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return False  # never finishes, you should use it with "withTimeout(...)"

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        self._drivetrain.stop()  # stop at the end

        super().end(interrupted)
