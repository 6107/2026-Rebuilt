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

from typing import Callable, List, Optional, Tuple

import commands2
from commands2 import InstantCommand
from wpilib import DriverStation, SmartDashboard
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from constants import MAX_SPEED, THETA_CONTROLLER_CONSTRAINTS
from lib_6107.commands.command import BaseCommand
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection
from lib_6107.commands.drivetrain.gotopoint import GoToPoint
from lib_6107.commands.drivetrain.swervetopoint import SwerveToPoint
from robot_2026.subsystems.swervedrive.constants import AutoConstants, DriveConstants
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem

FIELD_WIDTH = 8.052
FIELD_LENGTH = 17.55
U_TURN = Rotation2d.fromDegrees(180)


class JerkyTrajectory(BaseCommand):

    name = "JerkyTrajectory"

    def __init__(
            self,
            drivetrain: DriveSubsystem,
            endpoint: Pose2d | Translation2d | tuple | list | None,
            waypoints: List[Pose2d | Translation2d | tuple | list] = (),
            swerve: bool | str = False,
            speed=1.0,
            setup: Optional[Callable[[], None]] = None,
            stopAtEnd: bool = True,
            flipIfRed: bool = False,
    ):
        """
        A simple trajectory command that automatically skips all the waypoints that are already behind
        (this is why it is better to have start point should be listed in waypoints)
        :param swerve: do we want to use the swerve functionality (you can also say swerve="last-point")
        :param drivetrain: drive train, swerve or tank/arcade
        :param endpoint: end point of the trajectory, can be (X,Y) tuple, (X,Y,heading) tuple, Translation2d or Pose2d
        :param waypoints: a list of waypoints (if any), can be (X,Y) tuples, (X,Y,heading), Translation2d or Pose2d
        :param speed: maximum allowed driving speed (can be smaller than max speed of the drivetrain)
        """
        super().__init__(drivetrain)
        assert swerve in (
            False, True, "last-point"
        ), f"swerve={swerve} is not allowed (allowed: False, True, 'last-point')"

        if endpoint is None:
            assert len(waypoints) > 0, "if endpoint is None, waypoints cannot be empty"

        self.field = drivetrain.field if hasattr(drivetrain, "field") else None
        self.speed = speed
        self.swerve = swerve
        self.stopAtEnd = stopAtEnd
        self.flipIfRed = flipIfRed
        self.setup = setup
        self.waypoints = [self._makeWaypoint(w) for w in waypoints]
        if endpoint is not None:
            self.waypoints += [self._makeWaypoint(endpoint)]
        assert len(self.waypoints) > 0
        self.command = None

    def is_reversed(self):
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return JerkyTrajectory(
            self._drivetrain, endpoint, waypoints, self.swerve, -self.speed, self.setup, self.stopAtEnd, self.flipIfRed
        )

    def trajectoryToDisplay(self):
        result = []
        for translation, rotation in self.waypoints:
            result.append(Pose2d(translation, rotation if rotation is not None else Rotation2d()))
        return result

    def initialize(self):
        super().initialize()

        if self.setup is not None:
            self.setup()

        # skip the waypoints that are already behind
        waypoints = self.getRemainingWaypointsAheadOfUs()
        assert len(waypoints) > 0
        last = len(waypoints) - 1
        direction = None

        # make the commands connecting the waypoints which remain after skipping
        commands = []
        for index, (point, heading) in enumerate(waypoints):
            command = self._makeWaypointCommand(point, heading, index == last)
            direction = heading
            commands.append(command)

        # if the last waypoint (endpoint) has a specific direction, aim in that direction at the end
        if direction is not None:
            degrees = direction.degrees()
            log = lambda: SmartDashboard.putString(f"command/{self.getName()}", f"aim: {degrees}")
            aim = AimToDirection(self._drivetrain, degrees, turn_speed=self.speed).beforeStarting(log)
            commands.append(aim)

        # connect all these commands together and start
        self.command = commands2.SequentialCommandGroup(*commands)
        self.command.initialize()

    def getRemainingWaypointsAheadOfUs(self):
        mustFlip = False
        if self.flipIfRed:
            mustFlip = DriverStation.getAlliance() == DriverStation.Alliance.kRed

        SmartDashboard.putString(f"command/{self.getName()}", "running")

        # find the waypoint nearest to the current location: we want to skip all the waypoints before it
        nearest, distance, nh = None, None, None

        location = self._drivetrain.pose.translation()
        if mustFlip:  # if flipping the field for red team, flip robot location too
            location, _ = _flipWaypoint((location, None))

        for point, heading in self.waypoints:
            d = location.distance(point)
            if nearest is None or d < distance:
                nearest, distance, nh = point, d, heading

        # only use the waypoints that we must not skip (those past the nearest waypoint, i.e. not already behind)
        waypoints = []
        if nearest is not None:
            skip = True
            for point, heading in self.waypoints:
                if not skip:
                    waypoints.append((point, heading))
                elif point == nearest:
                    skip = False
        if len(waypoints) == 0:
            waypoints = [self.waypoints[-1]]

        # if the nearest point is also in the same direction, include it too
        if nearest is not None and _sameDirection(nearest - location, waypoints[0][0] - location):
            waypoints = [(nearest, nh)] + waypoints

        if mustFlip:
            waypoints = [_flipWaypoint(p) for p in waypoints]
        return waypoints

    def execute(self):
        if self.command is not None:
            self.command.execute()

    def isFinished(self) -> bool:
        if self.command is not None:
            return self.command.isFinished()

    def end(self, interrupted: bool):

        if self.command is not None:
            self.command.end(interrupted)
            self.command = None
        self._showTrajectory([])

        super().end(interrupted)

    def _makeWaypoint(self, waypoint):
        point, heading = None, None

        # did the user give us (X, Y) but not heading?
        if isinstance(waypoint, (tuple, list)) and len(waypoint) == 2:
            point, heading = waypoint[0], waypoint[1]
            if not isinstance(point, Translation2d):
                point = Translation2d(waypoint[0], waypoint[1])
                heading = None

        # did the user give us (X, Y) and heading?
        elif isinstance(waypoint, (tuple, list)) and len(waypoint) == 3:
            heading = waypoint[2]
            if isinstance(heading, (float, int)):
                heading = Rotation2d.fromDegrees(heading)
            point = Translation2d(waypoint[0], waypoint[1])

        # did the user just give us a Translation2d with (X, Y) inside?
        elif isinstance(waypoint, Translation2d):
            point = waypoint

        # did the user give us a Rotation2d with (X, Y, heading) inside?
        elif isinstance(waypoint, Pose2d):
            point = waypoint.translation()
            heading = waypoint.rotation()

        # here type(point) is Translation2d and type(heading) is Rotation2d or None
        return point, heading

    def _makeWaypointCommand(self, point, heading, last):
        slowdown = last and self.stopAtEnd
        if not self.swerve or (not last and self.swerve == "last-point"):
            # use arcade drive, if we aren't allowed to use swerve (or not allowed to use swerve for non-end points)
            command = GoToPoint(
                point.x, point.y, drivetrain=self._drivetrain, speed=self.speed, slow_down_at_finish=slowdown)
        else:
            command = SwerveToPoint(
                point.x, point.y, heading, drivetrain=self._drivetrain, speed=self.speed, slow_down_at_finish=slowdown)

        log = lambda: SmartDashboard.putString(f"command/{self.getName()}", f"next: {point.x}, {point.y}")
        return InstantCommand(log).andThen(command)

    def _showTrajectory(self, waypoints, chunks=10):
        if self.field is not None:
            trajectory = []
            prev = None
            for point, rotation in waypoints:
                if prev is None:
                    trajectory.append(Pose2d(point, U_TURN))
                else:
                    vector = point - prev
                    points = [prev + (vector * float((1 + chunk) / chunks)) for chunk in range(chunks)]
                    trajectory.extend([Pose2d(i, U_TURN) for i in points])
                prev = point
            self.field.getObject("active-trajectory").setPoses(trajectory)


class SwerveTrajectory(JerkyTrajectory):

    name = "SwerveTrajectory"

    def is_reversed(self):
        waypoints = self.waypoints[1:]
        waypoints.reverse()
        endpoint = self.waypoints[0]
        return SwerveTrajectory(
            self._drivetrain, endpoint, waypoints, self.swerve, -self.speed, self.setup, self.stopAtEnd, self.flipIfRed
        )

    def initialize(self):
        BaseCommand.initialize(self)

        if self.setup is not None:
            self.setup()

        # skip the waypoints that are already behind
        waypoints = self.getRemainingWaypointsAheadOfUs()
        assert len(waypoints) > 0
        end_pt, end_heading = waypoints[-1]

        import math
        from wpimath.trajectory import TrajectoryConfig, TrajectoryGenerator
        from wpimath.controller import PIDController, ProfiledPIDControllerRadians, HolonomicDriveController

        # Create config for trajectory
        config = TrajectoryConfig(
            MAX_SPEED * abs(self.speed),
            AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED,
        )
        # Add kinematics to ensure max speed is actually obeyed
        config.setKinematics(DriveConstants.DRIVE_KINEMATICS)

        current_pose = self._drivetrain.pose
        current_point, current_heading = current_pose.translation(), current_pose.rotation()
        if end_heading is None: end_heading = current_heading

        def makeTrajectoryPose(translation, rotation):
            if rotation is None:
                rotation = current_heading

            if self.speed < 0:
                rotation += Rotation2d.fromDegrees(180)

            return Pose2d(translation, rotation)

        # An example trajectory to follow. All units in meters.

        trajectory = TrajectoryGenerator.generateTrajectory(
            # Start at the current point
            makeTrajectoryPose(current_point, current_heading if not waypoints else waypoints[0][1]),
            # Pass through these interior waypoints
            [w[0] for w in waypoints[0:-1]],
            # End 1.5 meters straight ahead of where we started, facing forward
            makeTrajectoryPose(end_pt, end_heading),
            config,
        )
        theta_controller = ProfiledPIDControllerRadians(
            AutoConstants.P_THETA_CONTROLLER,
            0,
            0,
            THETA_CONTROLLER_CONSTRAINTS,
        )
        theta_controller.enableContinuousInput(-math.pi, math.pi)

        drive_controller = HolonomicDriveController(
            PIDController(AutoConstants.PX_CONTROLLER, 0, 0),
            PIDController(AutoConstants.PY_CONTROLLER, 0, 0),
            theta_controller,
        )

        swerve_controller_command = commands2.SwerveControllerCommand(
            trajectory,
            self._drivetrain.get_pose,  # Functional interface to feed supplier
            DriveConstants.DRIVE_KINEMATICS,
            drive_controller,
            lambda desired_states: self._drivetrain.set_module_states(desired_states),
            (self._drivetrain,),
            desiredRotation=lambda: end_heading
        )
        # If the robot fell behind the trajectory (trajectory speed too high in optimizer), brute-force catch up @ end
        catchup = SwerveToPoint(self._drivetrain, end_pt.x, end_pt.y, end_heading,
                                speed=self.speed, slow_down_at_finish=True
        ).beforeStarting(
            lambda: SmartDashboard.putString(f"command/{self.getName()}", f"catchup: {end_pt.x}, {end_pt.y}")
        )

        # Run path following command, then stop at the end (possibly turn in correct direction)
        if end_heading is not None:
            degrees = end_heading.degrees()
            log = lambda: SmartDashboard.putString(f"command/{self.getName()}", f"aim: {degrees}")
            stop = AimToDirection(self._drivetrain, degrees, turn_speed=self.speed).beforeStarting(log)
        else:
            stop = InstantCommand(self._drivetrain.stop, self._drivetrain)

        self.command = swerve_controller_command.andThen(catchup).andThen(stop)
        self.command.initialize()

        self._showTrajectory(waypoints)

def mirror(waypoints, width=FIELD_WIDTH):
    """
    Converts right-side trajectory into left-side trajectory
    :param waypoints: original trajectory, list of tuples of (x, y, heading) or (x, y)
    :param width: width of the field
    :return: a mirror image of trajectory waypoints
    """
    # a tuple is treated as a single waypoint
    if isinstance(waypoints, tuple):
        return mirror([waypoints])[0]

    def reflect(heading):
        if heading is not None:
            return heading * -1.0

    result = []
    for point in waypoints:
        if len(point) == 2 and isinstance(point[0], Translation2d):
            location, heading = point
            result.append((Translation2d(location.x, width - location.y), reflect(heading)))
        elif len(point) == 2:
            x, y = point
            result.append((x, width - y))
        elif len(point) == 3:
            x, y, heading = point
            result.append((x, width - y, reflect(heading)))
        else:
            assert False, f"unknown waypoint format: {point}"

    return result


def _flipWaypoint(waypoint, width=FIELD_WIDTH, length=FIELD_LENGTH) -> Tuple[Translation2d, Rotation2d]:
    translation, rotation = waypoint
    translation = Translation2d(length - translation.x, width - translation.y)
    if rotation is not None:
        rotation = rotation + U_TURN
    return translation, rotation


def _sameDirection(direction1: Translation2d, direction2: Translation2d, minCos=0.5) -> bool:
    """
    :param minCos: minimum cosine of angles between two directions (to be considered "same direction")
    """
    length1, length2 = direction1.norm(), direction2.norm()
    product = direction1.x * direction2.x + direction1.y * direction2.y
    return product > length1 * length2 * minCos
