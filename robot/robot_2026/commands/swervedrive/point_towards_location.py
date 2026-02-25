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
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

from commands2 import Command
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Translation2d

from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem


class PointTowardsLocation(Command):
    """
    One can use this command to have their swerve robot keep pointing towards some location as it moves around.
    Example (this can go into robotcontainer.py, inside configureButtonBindings() function):

        ```
            from commands.point_towards_location import PointTowardsLocation

            # create a command for keeping the robot nose pointed towards the hub
            keepPointingTowardsHub = PointTowardsLocation(
                drivetrain=self.robotDrive,
                location=Translation2d(x=4.59, y=4.025),
                locationIfRed=Translation2d(x=11.88, y=4.025),
            )

            # setup a condition for when to do this: do it when the joystick right trigger is pressed by more than 50%
            whenRightTriggerPressed = self.driverController.axisGreaterThan(
                XboxController.Axis.kRightTrigger, threshold=0.5
            )

            # connect the command to its trigger
            whenRightTriggerPressed.whileTrue(keepPointingTowardsHub)

        ```
    """
    def __init__(self, drivetrain: DriveSubsystem, blue_location: Translation2d, red_location: Translation2d):
        super().__init__()
        self._blue_location, self._red_location = blue_location, red_location
        self._drivetrain = drivetrain  # not calling addRequirement, on purpose

        self._active_target_location: Translation2d | None = None
        self._active = False

    def initialize(self):
        self._active = False
        color = DriverStation.getAlliance()

        if color == DriverStation.Alliance.kRed:
            self._active_target_location = self._red_location
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming red alliance")
        else:
            self._active_target_location = self._blue_location
            SmartDashboard.putString("command/c" + self.__class__.__name__, "assuming blue or unknown alliance")

    def execute(self):
        # heading override already in place?
        if self._active:
            return

        # try to place that heading override now
        if self._drivetrain.startOverrideToFaceThisPoint(self._active_target_location):
            self._active = True
            SmartDashboard.putString(
                "command/c" + self.__class__.__name__,
                f"pointing to x, y: {self._active_target_location.x}, {self._active_target_location.y}")

    def end(self, interrupted: bool):
        if self._active:
            self._drivetrain.stopOverrideToFaceThisPoint(self._active_target_location)
        SmartDashboard.putString("command/c" + self.__class__.__name__, "interrupted")

    def isFinished(self) -> bool:
        return False  # never finish, wait for user to stop this command
