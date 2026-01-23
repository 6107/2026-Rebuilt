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
# From Gene Panov's (Team 714) CommandRevSwerve project (and FRC Python videos)

from typing import Optional

from pathplannerlib.auto import NamedCommands
from wpimath.units import meters_per_second, radians_per_second

from lib_6107.commands.command import BaseCommand
from subsystems.swervedrive.drivesubsystem import DriveSubsystem


class ArcadeDrive(BaseCommand):

    def __init__(self, drivetrain: DriveSubsystem,
                 drive_speed: Optional[meters_per_second] = 0.0,
                 rotation_speed: Optional[radians_per_second] = 0.0,
                 assume_manual_input: Optional[bool] = False):
        """
        Drive the robot at `drive_speed` and `rotation_speed` until this command is terminated.
        """
        super().__init__(drivetrain)

        self._drive_speed = drive_speed
        if not callable(drive_speed):
            self._drive_speed = lambda: drive_speed

        self._rotation_speed = rotation_speed
        if not callable(rotation_speed):
            self._rotation_speed = lambda: rotation_speed

        self._start_time: float = 0
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
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def initialize(self):
        super().initialize()

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        drive_speed = self._drive_speed()        # get the drive speed from the joystick or wherever it comes from
        rotation_speed = self._rotation_speed()  # get the turn speed from the joystick or wherever it comes from
        self._drivetrain.arcade_drive(drive_speed,
                                      rotation_speed,
                                      assume_manual_input=self._assume_manual_input,
                                      field_relative=True)

    def end(self, interrupted: bool):
        self._drivetrain.stop()  # stop at the end
        super().end(interrupted)
