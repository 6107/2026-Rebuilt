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
import math
from typing import Callable, Optional

from commands2 import Command
from pathplannerlib.auto import NamedCommands
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d

from lib_6107.commands.command import BaseCommand
from subsystems.swervedrive.constants import AutoConstants
from subsystems.swervedrive.drivesubsystem import DriveSubsystem


class AimToDirectionConstants:
    kP = 0.001  # 0.002 is the default, but you must calibrate this to your robot
    USE_SQRT_CONTROL = AutoConstants.USE_SQRT_CONTROL

    MIN_TURN_SPEED = 0.025  # turning slower than this is unproductive for the motor (might not even spin)
    ANGLE_TOLERANCE_DEGREES = 4.0  # plus minus 3 degrees is "close enough"
    ANGLE_VELOCITY_TOLERANCE_DEGREES_PER_SEC = 1  # velocity under 100 degrees/second is considered "stopped"


class AimToDirection(BaseCommand):

    name = "AimToDirection"  # change this to something appropriate for this command

    def __init__(self, drivetrain: DriveSubsystem,
                 heading: Optional[Rotation2d | Callable[[], Rotation2d]] = None,
                 speed: Optional[float] = 1.0,
                 fwd_speed: Optional[float] = 0.0):
        super().__init__(drivetrain)

        self._speed = min((1.0, abs(speed)))
        self._target_direction = None
        self._fwd_speed = fwd_speed

        # setting the target angle in a way that works for all cases
        self._target_degrees = heading

        if heading is None:
            self._target_degrees = lambda: self._drivetrain.heading.degrees()

        elif not callable(heading):
            self._target_degrees = lambda: heading

    @staticmethod
    def pathplanner_register(drivetrain: DriveSubsystem) -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> Command:
            return AimToDirection(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def initialize(self):
        super().initialize()

        self._target_direction = Rotation2d.fromDegrees(self._target_degrees())

    def execute(self):
        # 1. how many degrees are left to turn?
        current_direction = self._drivetrain.heading
        rotation_remaining = self._target_direction - current_direction
        degrees_remaining = rotation_remaining.degrees()

        # (do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degrees_remaining > 180:
            degrees_remaining -= 360

        while degrees_remaining < -180:
            degrees_remaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        turn_speed = self._speed
        proportional_speed = AimToDirectionConstants.kP * abs(degrees_remaining)

        if AimToDirectionConstants.USE_SQRT_CONTROL:
            proportional_speed = math.sqrt(0.5 * proportional_speed)  # will match the non-sqrt value when 50% max speed

        if turn_speed > proportional_speed:
            turn_speed = proportional_speed

        if turn_speed < AimToDirectionConstants.MIN_TURN_SPEED and self._fwd_speed == 0:
            turn_speed = AimToDirectionConstants.MIN_TURN_SPEED  # but not too small

        # 3. act on it! if target angle is on the right, turn right
        if degrees_remaining > 0:
            self._drivetrain.arcade_drive(self._fwd_speed, +turn_speed)
        else:
            self._drivetrain.arcade_drive(self._fwd_speed, -turn_speed)  # otherwise, turn left

    def end(self, interrupted: bool):
        self._drivetrain.stop()

        super().end(interrupted)

    def isFinished(self) -> bool:
        if self._fwd_speed != 0:
            return False  # if someone wants us to drive forward while aiming, then we are never finished

        current_direction = self._drivetrain.heading
        rotation_remaining = self._target_direction - current_direction
        degrees_remaining = rotation_remaining.degrees()

        # if we are pretty close to the direction we wanted, consider the command finished
        if abs(degrees_remaining) < AimToDirectionConstants.ANGLE_TOLERANCE_DEGREES:
            turn_velocity = self._drivetrain.gyro.turn_rate_degrees_per_second
            SmartDashboard.putString(f"{self.getName()}", "good angle")

            if abs(turn_velocity) < AimToDirectionConstants.ANGLE_VELOCITY_TOLERANCE_DEGREES_PER_SEC:
                SmartDashboard.putString(f"{self.getName()}", "completed")
                return True

        return False
