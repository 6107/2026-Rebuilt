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
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from typing import Optional

from pathplannerlib.auto import NamedCommands
from wpilib import SmartDashboard
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import degrees

from constants import MAX_SPEED
from lib_6107.commands.command import BaseCommand
from lib_6107.commands.drivetrain.aimtodirection import AimToDirectionConstants
from robot_2026.subsystems.swervedrive.constants import AutoConstants
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem


class GoToPointConstants:
    KP_TRANSLATE = 0.25 / MAX_SPEED / 4.7
    USE_SQRT_CONTROL = AutoConstants.USE_SQRT_CONTROL

    MIN_TRANSLATE_SPEED = 0.035  # moving forward slower than this is unproductive
    APPROACH_RADIUS = 0.2  # within this radius from target location, try to point in desired direction
    OVERSTEER_ADJUSTMENT = 0.5


class GoToPoint(BaseCommand):
    """
    Go to a specific point on the field.

    TODO: Add field limitations to limit x & y and calculate in robot size
    TODO: Add pathplanner support (optional) and have pathplanner determine the path to take
    """
    name = "GoToPoint"

    def __init__(self, drivetrain: DriveSubsystem,
                 x: Optional[int | float] = 0,
                 y: Optional[int | float] = 0,
                 speed: Optional[float] = 1.0,
                 slow_down_at_finish: Optional[bool] = True,
                 finish_direction: Optional[Rotation2d] = None) -> None:
        """
        Go to a point with (X, Y) coordinates. Whether this is the end of your trajectory or not.
        :param x:
        :param y:
        :param drivetrain:
        :param speed: between -1.0 and +1.0 (you can use negative speed to drive backwards)
        :param finish_direction: Rotation2d for robot direction at the finish point, example: Rotation2d.fromDegrees(-70)
        :param slow_down_at_finish:
        """
        super().__init__(drivetrain)

        self._target_position = Translation2d(x, y)
        self._initial_position = None
        self._speed = min(1.0, max(0.0, speed))
        self._stop = slow_down_at_finish
        self._desired_end_direction = None
        self._initial_distance = None
        self._pointing_in_good_direction = False

        self._finish_direction = finish_direction

        if self._speed < 0 and self._finish_direction is not None:
            self._finish_direction = self._finish_direction.rotateBy(GoToPoint.REVERSE_DIRECTION)

    @staticmethod
    def pathplanner_register(drivetrain: DriveSubsystem) -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> GoToPoint:
            return GoToPoint(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(command().name, command())

    def initialize(self):
        """
        Called just before this Command runs the first time
        """
        super().initialize()

        self._initial_position = self._drivetrain.pose.translation()

        if self._finish_direction is not None:
            self._desired_end_direction = self._finish_direction
        else:
            initial_direction = self._target_position - self._initial_position
            self._desired_end_direction = Rotation2d(initial_direction.x, initial_direction.y)

        if self._speed < 0:
            self._desired_end_direction = self._desired_end_direction.rotateBy(GoToPoint.REVERSE_DIRECTION)

        self._initial_distance = self._initial_position.distance(self._target_position)
        self._pointing_in_good_direction = False

    def execute(self):
        """
        The main body of a command. Called repeatedly while the command is scheduled.
        """
        # 1. to which direction we should be pointing?
        current_pose = self._drivetrain.pose
        current_direction = current_pose.rotation()
        current_point = current_pose.translation()
        target_direction_vector = self._target_position - current_point
        target_direction = Rotation2d(target_direction_vector.x, target_direction_vector.y)

        if self._speed < 0:
            target_direction = target_direction.rotateBy(GoToPoint.REVERSE_DIRECTION)

        degrees_remaining = _optimize((target_direction - current_direction).degrees())
        rotate_speed = min([abs(self._speed), AimToDirectionConstants.kP * abs(degrees_remaining)])

        # 2. if we are pointing in a very wrong direction (more than 45 degrees away), rotate away without moving
        if degrees_remaining > 45 and not self._pointing_in_good_direction:
            self._drivetrain.arcade_drive(0.0, rotate_speed)
            return

        if degrees_remaining < -45 and not self._pointing_in_good_direction:
            self._drivetrain.arcade_drive(0.0, -rotate_speed)
            return

        if not self._pointing_in_good_direction:
            SmartDashboard.putString(f"command/{self.getName()}", "in good direction")
            self._pointing_in_good_direction = True

        # 3. otherwise, drive forward but with an oversteer adjustment
        #    TODO: (better way is to use RAMSETE unicycle or the LTV Unicycle Controller)
        distance_remaining = self._target_position.distance(current_point)

        if distance_remaining < GoToPointConstants.APPROACH_RADIUS:
            target_direction = self._desired_end_direction  # avoid wiggling the direction when almost there
            degrees_remaining = _optimize((target_direction - current_direction).degrees())

        elif GoToPointConstants.OVERSTEER_ADJUSTMENT != 0:
            deviation_from_initial = _optimize((target_direction - self._desired_end_direction).degrees())
            adjustment = GoToPointConstants.OVERSTEER_ADJUSTMENT * deviation_from_initial

            if adjustment > 30:
                adjustment = 30  # avoid oscillations by capping the adjustment at 30 degrees

            if adjustment < -30:
                adjustment = -30  # avoid oscillations by capping the adjustment at 30 degrees

            target_direction = target_direction.rotateBy(Rotation2d.fromDegrees(adjustment))
            degrees_remaining = _optimize((target_direction - current_direction).degrees())
            # SmartDashboard.putNumber("z-heading-target", targetDirection.degrees())

        # 4. now when we know the desired direction, we can compute the turn speed
        rotate_speed = abs(self._speed)
        proportional_rotate_speed = AimToDirectionConstants.kP * abs(degrees_remaining)

        if AimToDirectionConstants.USE_SQRT_CONTROL:
            proportional_rotate_speed = math.sqrt(0.5 * proportional_rotate_speed)  # will match the non-sqrt value when 50% max speed

        if rotate_speed > proportional_rotate_speed:
            rotate_speed = proportional_rotate_speed

        # 5. but if not too different, then we can drive while turning
        proportional_trans_speed = GoToPointConstants.KP_TRANSLATE * distance_remaining

        if GoToPointConstants.USE_SQRT_CONTROL:
            proportional_trans_speed = math.sqrt(0.5 * proportional_trans_speed)

        translate_speed = abs(self._speed)  # if we don't plan to stop at the end, go at max speed

        if translate_speed > proportional_trans_speed and self._stop:
            translate_speed = proportional_trans_speed  # if we plan to stop at the end, slow down when close

        if translate_speed < GoToPointConstants.MIN_TRANSLATE_SPEED:
            translate_speed = GoToPointConstants.MIN_TRANSLATE_SPEED

        if self._speed < 0:
            translate_speed = -translate_speed  # negative translation speed if supposed to go in reverse

        # 6. if we need to be turning *right* while driving, use negative rotation speed
        if degrees_remaining < 0:
            self._drivetrain.arcade_drive(translate_speed, -rotate_speed)

        else:  # otherwise, use positive
            self._drivetrain.arcade_drive(translate_speed, +rotate_speed)

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        # 1. did we reach the point where we must move very slow?
        current_pose = self._drivetrain.pose
        current_position = current_pose.translation()
        distance_from_initial_position = self._initial_position.distance(current_position)

        if not self._stop and distance_from_initial_position > self._initial_distance - GoToPointConstants.APPROACH_RADIUS:
            SmartDashboard.putString(f"command/{self.getName()}", "close enough")
            return True  # close enough

        distance_remaining = self._target_position.distance(current_position)
        translate_speed = GoToPointConstants.KP_TRANSLATE * distance_remaining

        if GoToPointConstants.USE_SQRT_CONTROL:
            translate_speed = math.sqrt(0.5 * translate_speed)

        # 1. have we reached the point where we are moving very slowly?
        too_slow_now = translate_speed < 0.125 * GoToPointConstants.MIN_TRANSLATE_SPEED and self._stop

        # 2. did we overshoot?
        if distance_from_initial_position >= self._initial_distance:
            SmartDashboard.putString(f"command/{self.getName()}", "overshot")
            return True  # we overshot or driving too slow

        if too_slow_now:
            SmartDashboard.putString(f"command/{self.getName()}", "slow enough")
            return True

        return False

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        self._drivetrain.stop()
        super().end(interrupted)


    REVERSE_DIRECTION = Rotation2d.fromDegrees(180)


def _optimize(deg) -> degrees:
    while deg > 180:  # for example, if we have 350 degrees to turn left, we probably want -10 degrees right
        deg -= 360

    while deg < -180:  # for example, if we have -350 degrees to turn right, we probably want +10 degrees left
        deg += 360

    return deg
