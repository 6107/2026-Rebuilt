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
from typing import Optional, Callable

from commands2 import Command
from pathplannerlib.auto import NamedCommands
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d, Pose2d
from wpimath.units import degrees, meters

from lib_6107.commands.command import BaseCommand
from lib_6107.commands.drivetrain.aimtodirection import AimToDirectionConstants
from lib_6107.commands.drivetrain.gotopoint import GoToPointConstants


class SwerveToPoint(BaseCommand):

    def __init__(self,
                 drivetrain: 'DriveSubsystem',
                 x:  Optional[int | float] = 0,
                 y:  Optional[int | float] = 0,
                 heading: Optional[Rotation2d | degrees] = 0.0,
                 speed: Optional[float] = 1.0,
                 slow_down_at_finish: Optional[bool] = True,
                 rate_limit: Optional[bool] = False) -> None:
        super().__init__(drivetrain)

        self._target_pose = None
        self._target_point = Translation2d(x, y)
        heading = heading or Rotation2d(0)

        if isinstance(heading, Rotation2d):
            self._target_heading = heading

        elif heading is not None:
            self._target_heading = Rotation2d.fromDegrees(heading)
        else:
            self._target_heading = None

        self._speed = speed
        self._stop = slow_down_at_finish
        self._rate_limit = rate_limit

        self._initial_position = None
        self._initial_distance = None
        self._overshot = False

    @staticmethod
    def pathplanner_register(drivetrain: 'DriveSubsystem') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> Command:
            return SwerveToPoint(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def initialize(self):
        super().initialize()

        initial_pose = self._drivetrain.pose
        self._initial_position = initial_pose.translation()

        target_heading = initial_pose.rotation() if self._target_heading is None else self._target_heading
        self._target_pose = Pose2d(self._target_point, target_heading)

        self._initial_distance = self._initial_position.distance(self._target_pose.translation())
        self._overshot = False

    def execute(self):
        current_xy = self._drivetrain.pose
        x_distance, y_distance = self._target_pose.x - current_xy.x, self._target_pose.y - current_xy.y
        total_distance = self._target_pose.translation().distance(current_xy.translation())

        total_speed = abs(self._speed)
        if self._stop:  # proportional control: start slowing down if close to finish
            total_speed = GoToPointConstants.KP_TRANSLATE * total_distance
            if GoToPointConstants.USE_SQRT_CONTROL:
                total_speed = math.sqrt(0.5 * total_speed)

        if total_speed > abs(self._speed):
            total_speed = abs(self._speed)

        if total_speed < GoToPointConstants.MIN_TRANSLATE_SPEED:
            total_speed = GoToPointConstants.MIN_TRANSLATE_SPEED

        # distribute the total speed between x speed and y speed
        x_speed, y_speed = 0, 0

        if total_distance > 0:
            x_speed = total_speed * x_distance / total_distance
            y_speed = total_speed * y_distance / total_distance

        degrees_left_to_turn = self.get_degrees_left_to_turn()
        turning_speed = abs(degrees_left_to_turn) * AimToDirectionConstants.kP

        if AimToDirectionConstants.USE_SQRT_CONTROL:
            turning_speed = math.sqrt(0.5 * turning_speed)  # will match the non-sqrt value when 50% max speed

        if turning_speed > abs(self._speed):
            turning_speed = abs(self._speed)

        if degrees_left_to_turn < 0:
            turning_speed = -turning_speed

        # TODO: Make sure parameters are meters_per_second and radians_per_second
        # now rotate xSpeed and ySpeed into robot coordinates
        speed = Translation2d(x=x_speed, y=y_speed).rotateBy(-self._drivetrain.heading)

        self._drivetrain.drive(speed.x, speed.y, turning_speed, field_relative=False, rate_limit=self._rate_limit)

    def end(self, interrupted: bool):
        self._drivetrain.stop()
        super().end(interrupted)

    def isFinished(self) -> bool:
        current_pose = self._drivetrain.pose
        current_position = current_pose.translation()

        # did we overshoot?
        distance_from_initial_position = self._initial_position.distance(current_position)

        if not self._stop and distance_from_initial_position > self._initial_distance - GoToPointConstants.APPROACH_RADIUS:
            SmartDashboard.putString(f"command/{self.getName()}", "acceptable")
            return True  # close enough

        if distance_from_initial_position > self._initial_distance:
            if not self._overshot:
                SmartDashboard.putString(f"command/{self.getName()}", "overshooting")

            self._overshot = True

        if self._overshot:
            distance_from_target_direction_degrees = self.get_degrees_left_to_turn()

            if abs(distance_from_target_direction_degrees) < 3 * AimToDirectionConstants.ANGLE_TOLERANCE_DEGREES:
                SmartDashboard.putString(f"command/{self.getName()}", "completed")
                return True  # case 2: overshot in distance and target direction is correct

        return False

    def get_degrees_left_to_turn(self):
        # can we get rid of this function by using Rotation2d? probably we can

        current_heading = self._drivetrain.pose.rotation()
        degrees_left_to_turn = (self._target_pose.rotation() - current_heading).degrees()

        # if we have +350 degrees left to turn, this really means we have -10 degrees left to turn
        while degrees_left_to_turn > 180:
            degrees_left_to_turn -= 360

        # if we have -350 degrees left to turn, this really means we have +10 degrees left to turn
        while degrees_left_to_turn < -180:
            degrees_left_to_turn += 360

        return degrees_left_to_turn


class SwerveMove(BaseCommand):
    def __init__(
            self,
            drivetrain: 'DriveSubsystem',
            meters_to_the_left: Optional[meters] = 0.0,
            meters_backwards: Optional[meters] = 0.0,
            speed: Optional[float] = 1.0,
            heading: Optional[Rotation2d, Callable[[], Rotation2d]] = None,
            slow_down_at_finish: Optional[bool] = True) -> None:

        super().__init__(drivetrain)

        self._speed = speed
        self._meters_to_the_left = meters_to_the_left
        self._meters_backward = meters_backwards
        self._slow_down_at_finish = slow_down_at_finish

        self._desired_heading = heading
        if heading is not None and not callable(heading):
            self._desired_heading = lambda: heading

        self._subcommand = None

    @staticmethod
    def pathplanner_register(drivetrain: 'DriveSubsystem') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> Command:
            return SwerveMove(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def initialize(self):
        super().initialize()

        position = self._drivetrain.pose
        heading = self._desired_heading() if self._desired_heading is not None else position.rotation()

        target = position.translation() + Translation2d(x=-self._meters_backward,
                                                        y=self._meters_to_the_left).rotateBy(heading)

        self._subcommand = SwerveToPoint(x=target.x, y=target.y,
                                         heading=heading.degrees(),
                                         drivetrain=self._drivetrain,
                                         speed=self._speed,
                                         slow_down_at_finish=self._slow_down_at_finish)
        self._subcommand.initialize()

    def isFinished(self) -> bool:
        return self._subcommand.isFinished()

    def execute(self):
        return self._subcommand.execute()

    def end(self, interrupted: bool):
        self._subcommand.end(interrupted)
        super().end(interrupted)

