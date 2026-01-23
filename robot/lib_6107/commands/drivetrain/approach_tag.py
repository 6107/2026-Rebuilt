#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import math
from typing import Optional, Callable, Any

from commands2 import Command
from pathplannerlib.auto import NamedCommands
from wpilib import Timer, SmartDashboard, SendableChooser
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.units import seconds, meters, percent

from lib_6107.commands.command import BaseCommand
from lib_6107.commands.drivetrain.aimtodirection import AimToDirectionConstants
from lib_6107.commands.drivetrain.gotopoint import GoToPointConstants
from subsystems.swervedrive.drivesubsystem import DriveSubsystem


class Tunable:
    _choosers = {}

    def __init__(self, settings, prefix, name, default, min_max_range):
        if settings is not None:
            if name in settings:
                self.value = settings[name]
                self.chooser = None
                return

            if (prefix + name) in settings:
                self.value = settings[prefix + name]
                self.chooser = None
                return

        self.value = None
        self.chooser = Tunable._choosers.get(prefix + name)
        if self.chooser is not None:
            return

        # if that chooser was not created yet, create it now
        Tunable._choosers[prefix + name] = self.chooser = SendableChooser()

        for index, factor in enumerate([0, 0.1, 0.17, 0.25, 0.35, 0.5, 0.7, 1.0, 1.4, 2.0, 2.8, 4.0]):
            label, value = f"{factor * default}", factor * default

            if min_max_range[0] <= value <= min_max_range[1]:

                if factor == 1.0:
                    self.chooser.setDefaultOption(label, value)
                else:
                    self.chooser.addOption(label, value)

        SmartDashboard.putData(prefix + name, self.chooser)

    def fetch(self):
        if self.chooser is not None:
            self.value = self.chooser.getSelected()

    def __call__(self, *args, **kwargs):
        self.fetch()
        return self.value


class ApproachTag(BaseCommand):
    def __init__(self, drivetrain: DriveSubsystem,
                 camera: Optional[Any] = None,
                 specific_heading: Optional[Rotation2d | Callable[[], Rotation2d]] = None,
                 speed: Optional[float]=1.0,
                 reverse: Optional[bool]=False,
                 settings: Optional[dict | None] = None,
                 push_forward: Optional[seconds | Callable[[], seconds]] = 0.0,  # length of final approach
                 push_forward_min_distance: Optional[meters] = 0.0,  # length of final approach in minimum distance
                 final_approach_obj_size: Optional[percent] = 10.0,
                 detection_timeout: Optional[seconds] = 2.0,
                 camera_minimum_fps: Optional[int | float] = 4.0,
                 dashboard_name: Optional[str] = ""):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specific_heading: do you want the robot to face in a very specific direction? then specify it
        :param speed: positive speed, even if camera is on the back of your robot (for the latter case set reverse=True)
        :param push_forward: if you want the robot to do some kind of final approach at the end of alignment
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param detection_timeout: if no detection within this many seconds, assume the tag is lost
        :param camera_minimum_fps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__(drivetrain)
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"

        self._camera = camera or drivetrain.front_camera
        self.addRequirements(camera)

        self._start_time: float = 0
        self._reverse = reverse
        self._approach_speed = min((1.0, abs(speed)))  # ensure that the speed is between 0.0 and 1.0
        self._final_approach_object_size = final_approach_obj_size

        self._final_approach_min_distance = push_forward_min_distance
        self._push_forward_seconds = push_forward

        if self._push_forward_seconds is None:
            self._push_forward_seconds = Tunable(settings, dashboard_name, "BrakeDst", 1.0, (0.0, 10.0))

        elif not callable(self._push_forward_seconds):
            self._push_forward_seconds = lambda: push_forward

        self._final_approach_speed = None
        self._tag_to_final_approach_point = None  # will be assigned in initialize()

        assert detection_timeout > 0, f"non-positive detectionTimeoutSeconds={detection_timeout}"
        self._detection_timeout = detection_timeout

        assert camera_minimum_fps > 0, f"non-positive cameraMinimumFps={camera_minimum_fps}"
        self._frame_timeout: seconds = 1.0 / camera_minimum_fps

        # setting the target heading in a way that works for all cases
        self._target_degrees = specific_heading

        if specific_heading is None:
            self._target_degrees = lambda: self._drivetrain.heading.degrees()

        elif not callable(specific_heading):
            self._target_degrees = lambda: specific_heading

        # state
        self._target_direction = None
        self._last_seen_object_time = None
        self._last_seed_object_x = 0.0
        self._last_seed_object_size = 0.0
        self._last_seed_distance_to_tag = None
        self._ever_saw_object = False
        self._reached_glide_path_time = 0.0  # time when aligned to the tag and desired direction for the first time
        self._reached_final_approach_time = 0.0  # time when reached the final approach
        self._reached_final_approach_xy = Translation2d(0, 0)
        self._lost_tag = ""
        self._finished = ""

        # debugging
        self._start_time = 0
        self._last_state = self.get_state()
        self._last_warnings = None

        self.init_tunables(settings, dashboard_name)

    @staticmethod
    def pathplanner_register(drivetrain: 'DriveSubsystem') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> Command:
            return ApproachTag(drivetrain, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def isReady(self, min_required_object_size=0.3):
        return self._camera.hasDetection() and self._camera.getA() > min_required_object_size

    def init_tunables(self, settings, prefix):
        self.KPMULT_TRANSLATION = Tunable(settings, prefix, "GainTran", 0.5, (0.1, 8.0))  # gain for how quickly to move
        self.KPMULT_ROTATION = Tunable(settings, prefix, "GainRot", 0.8, (0.1, 8.0))  # gail for how quickly to rotate

        # acceptable width of glide path, in inches
        self.GLIDE_PATH_WIDTH_INCHES = Tunable(settings, prefix, "Tolernce", 2.0, (0.5, 4.0))

        # if plus minus 30 degrees from desired heading, forward motion is allowed before final approach
        self.DESIRED_HEADING_RADIUS = Tunable(settings, prefix, "Headng+-", 30, (5, 45))

        # shape pre final approach: 2 = use parabola for before-final-approach trajectory, 3.0 = use cubic curve, etc.
        self.APPROACH_SHAPE = Tunable(settings, prefix, "TrjShape", 3.0, (2.0, 8.0))

        self.OUT_OF_SIGHT_ALLOWED = Tunable(settings, prefix, "AllowOOS", 1.0, (0.0, 1.0))

        self.tunables = [
            self.GLIDE_PATH_WIDTH_INCHES,
            self.DESIRED_HEADING_RADIUS,
            self.KPMULT_TRANSLATION,
            self.KPMULT_ROTATION,
            self.APPROACH_SHAPE,
            self.OUT_OF_SIGHT_ALLOWED,
        ]
        if isinstance(self._push_forward_seconds, Tunable):
            self.tunables.append(self._push_forward_seconds)

    def initialize(self):
        super().initialize()

        for t in self.tunables:
            t.fetch()

        kp_mult_tran = self.KPMULT_TRANSLATION.value
        # print(f"ApproachTag: translation gain value {kp_mult_tran}, power={self.APPROACH_SHAPE.value}")

        target_degrees = self._target_degrees()
        if target_degrees is None:
            target_degrees = self._drivetrain.heading.degrees()

        self._target_direction = Rotation2d.fromDegrees(target_degrees)
        self._reached_glide_path_time = 0.0  # time when reached the glide path
        self._reached_final_approach_time = 0.0  # time when reached the final approach
        self._reached_final_approach_xy = Translation2d(0, 0)
        self._lost_tag = False
        self._last_seed_object_x = 0.0
        self._last_seed_object_size = 0.0
        self._last_seed_distance_to_tag = 999
        self._last_seen_object_time = Timer.getFPGATimestamp()
        self._ever_saw_object = False
        self._finished = ""

        # final approach parameters
        self._tag_to_final_approach_point = self.compute_tag_distance_from_tag_size_on_frame(self._final_approach_object_size)
        self._final_approach_speed = 0

        self._final_approach_seconds = max([0, self._push_forward_seconds()])
        if self._final_approach_seconds > 0:
            self._final_approach_speed = self.compute_proportional_speed(self._tag_to_final_approach_point)

        # debugging info
        self._start_time = Timer.getFPGATimestamp()
        self._last_state = -1
        self._last_warnings = None

    def isFinished(self) -> bool:
        if self._finished:
            return True

        now = Timer.getFPGATimestamp()

        # bad ways to finish
        if self._lost_tag:
            self._finished = self._lost_tag

        elif now > self._last_seen_object_time + self._detection_timeout + self._final_approach_seconds:
            delay = now - self._last_seen_object_time
            self._finished = f"not seen {int(1000 * delay)}ms"

        # good ways to finish
        elif self._reached_final_approach_time != 0:
            length = (self._drivetrain.pose.translation() - self._reached_final_approach_xy).norm()

            if now > self._reached_final_approach_time + self._final_approach_seconds:
                self._finished = f"approached within {now - self._reached_final_approach_time}s, drove {length}m"

        if not self._finished:
            return False

        return True

    def end(self, interrupted: bool):
        self._drivetrain.stop()
        super().end(interrupted)

    def execute(self):
        now = Timer.getFPGATimestamp()

        # 0. look at the camera
        self.update_vision(now)
        vision_old = (now - self._last_seen_object_time) / (0.5 * self._frame_timeout)

        if self._lost_tag:
            self._drivetrain.stop()
            return

        # 1. how many degrees are left to turn? (and recommended rotation speed)
        rotation_speed, degrees_left_to_rotate = self.get_gyro_based_rotation_speed()

        # 2. how far from the glide path? (and recommended translation speed)
        fwd_speed, left_speed, distance_to_glide_path = self.get_vision_based_swerve_speed(now)

        # 3. have we reached the glide path?
        if self.has_reached_glide_path(degrees_left_to_rotate, distance_to_glide_path):
            if self._reached_glide_path_time == 0:
                self._reached_glide_path_time = now

        # 4. be careful with forward speed
        warnings = None

        if self._reached_final_approach_time != 0:
            # - if we are on final approach, completely ignore the fwdSpeed from the visual estimation
            fwd_speed = 0
            if self._final_approach_seconds > 0:
                completed_percentage = (now - self._reached_final_approach_time) / self._final_approach_seconds

                if self._final_approach_min_distance > 0:
                    completed_distance = (self._drivetrain.pose.translation() - self._reached_final_approach_xy).norm()

                    if completed_distance < self._final_approach_min_distance:
                        completed_percentage = 0.0  # if min distance is not met, don't even slow down

                fwd_speed = self._final_approach_speed * max((0.0, 1.0 - completed_percentage))

                if abs(fwd_speed) < GoToPointConstants.MIN_TRANSLATE_SPEED:
                    fwd_speed = math.copysign(GoToPointConstants.MIN_TRANSLATE_SPEED, self._final_approach_speed)

            left_speed *= max(0.0, 1 - vision_old * vision_old)  # final approach: dial down the left speed if no object
        else:
            # - slow down if the visual estimate is old, if heading is not right yet, or if rotating away
            close_to_edge = 0

            if self._ever_saw_object and self.OUT_OF_SIGHT_ALLOWED.value == 0 and self._last_seed_object_x * rotation_speed > 0:
                close_to_edge = abs(self._last_seed_object_x) / 5.0  # rotating away from the object in frame? slow this down!

            far_from_desired_heading = abs(degrees_left_to_rotate) / self.DESIRED_HEADING_RADIUS.value

            if far_from_desired_heading >= 1:
                warnings = "large heading error"

            if close_to_edge >= 1:
                warnings = "close to frame edge"

            if vision_old >= 1:
                warnings = f"temporarily out of sight"

            # any other reason to slow down? put it above
            fwd_speed *= max(0.0, 1 - max(far_from_desired_heading, close_to_edge, vision_old))

            if self.OUT_OF_SIGHT_ALLOWED.value == 0:
                left_speed *= max(0.0, 1 - vision_old)
                rotation_speed *= max(0.0, 1 - max(vision_old, close_to_edge))

        # 5. drive!
        if self._reverse:     # TODO: Make sure parameters are meters_per_second and radians_per_second
            self._drivetrain.drive(-fwd_speed, -left_speed, rotation_speed, field_relative=False, rate_limit=False)
        else:
            self._drivetrain.drive(fwd_speed, left_speed, rotation_speed, field_relative=False, rate_limit=False)

        # 6. debug
        state = self.get_state()

        if state != self._last_state or warnings != self._last_warnings:
            SmartDashboard.putString(f"command/{self.getName()}", warnings or self.STATE_NAMES[state])

        self._last_state = state
        self._last_warnings = warnings

    def get_gyro_based_rotation_speed(self):
        # 1. how many degrees are left to turn?
        current_direction = self._drivetrain.heading
        rotation_remaining = self._target_direction - current_direction
        degrees_remaining = rotation_remaining.degrees()

        # (optimize: do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degrees_remaining > 180:
            degrees_remaining -= 360

        while degrees_remaining < -180:
            degrees_remaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportional_speed = self.KPMULT_ROTATION.value * AimToDirectionConstants.kP * abs(degrees_remaining)
        proportional_speed = math.sqrt(0.5 * proportional_speed)  # will match the non-sqrt value when 50% max speed

        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        turn_speed = min([proportional_speed, 1.0])
        if degrees_remaining < 0:
            turn_speed = -turn_speed

        return turn_speed, degrees_remaining

    def get_vision_based_swerve_speed(self, now):
        direction = self.get_vision_based_swerve_direction(now)
        if direction is None:
            return 0.0, 0.0, None

        # use proportional control to compute the velocity
        distance = direction.norm()
        velocity = self.compute_proportional_speed(distance)

        # adjust the direction to account for APPROACH_SHAPE_POWER (= how strongly we prioritize getting to glide path)
        direction = Translation2d(direction.x, direction.y * self.APPROACH_SHAPE.value)
        norm = direction.norm()

        # distribute velocity between X and Y velocities in a way that gives us correct trajectory shape
        y_velocity = velocity * (direction.y / norm)
        x_velocity = velocity * (direction.x / norm)

        # do we need to rescale these velocities to meet constraints?
        norm = Translation2d(x_velocity, y_velocity).norm()

        if norm < GoToPointConstants.MIN_TRANSLATE_SPEED:
            factor = GoToPointConstants.MIN_TRANSLATE_SPEED / norm
            x_velocity *= factor
            y_velocity *= factor

        # done
        return x_velocity, y_velocity, abs(direction.y)

    def get_vision_based_swerve_direction(self, now):
        # can we trust the last seen object?
        if not (self._last_seed_object_size > 0):
            return None  # the object is not yet there, hoping that this is temporary

        # where are we?
        robot_x, robot_y, tag_x = self.localize()

        # have we reached the final approach point now? (must already be on glide path, otherwise it doesn't count)
        if self._reached_glide_path_time != 0 and self._reached_final_approach_time == 0 and robot_x > 0:
            SmartDashboard.putString(f"command/{self.getName()}", "reached final approach")
            self._reached_final_approach_time = now
            self._reached_final_approach_xy = self._drivetrain.pose.translation()
            print(f"final approach starting from {self._reached_final_approach_xy}")

        # if we already reached the glide path, and we want nonzero final approach (after reaching desired size)
        # ... then go directly towards the tag (x, y = tagX, 0) instead of going towards 0, 0
        if self._reached_glide_path_time != 0 and self._final_approach_seconds > 0:
            direction = Translation2d(x=tag_x - robot_x, y=0.0 - robot_y)
        else:
            direction = Translation2d(x=0.0 - robot_x, y=0.0 - robot_y)  # otherwise go towards 0, 0

        if not (direction.x != 0 or direction.y != 0):
            SmartDashboard.putString(f"command/{self.getName()}", "warning: distance not positive")
            return None

        return direction

    def localize(self):
        """
        localize the robot camera in the frame of the final approach point
        (i.e. final approach point is assumed to be at (x, y) = (0, 0), tag is assumed to be at (x, y) = (d, 0))
        :return: (x, y, d), where `x,y` are coordinates of the robot and `d` is distance between that point and tag
        """
        distance_to_tag = self._last_seed_distance_to_tag

        # trigonometry: how many meters on the left is our tag? (if negative, then it's on the right)
        angle = Rotation2d.fromDegrees(self._last_seed_object_x)
        y = -distance_to_tag * angle.sin()

        distance_to_final_approach = distance_to_tag - self._tag_to_final_approach_point
        return -distance_to_final_approach, -y, self._tag_to_final_approach_point

    def compute_proportional_speed(self, distance) -> float:
        kp_mult_tran = self.KPMULT_TRANSLATION.value
        velocity = distance * GoToPointConstants.KP_TRANSLATE * kp_mult_tran
        velocity = math.sqrt(0.5 * velocity * kp_mult_tran)

        if velocity > self._approach_speed:
            velocity = self._approach_speed

        if velocity < GoToPointConstants.MIN_TRANSLATE_SPEED:
            velocity = GoToPointConstants.MIN_TRANSLATE_SPEED

        return velocity

    @staticmethod
    def compute_tag_distance_from_tag_size_on_frame(object_size: percent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        return math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * object_size))
        # note: Arducam w OV9281 (and Limelight 3 / 4) is 0.57 sq radians (not 1.33)

    def has_reached_glide_path(self, degrees_left_to_rotate: float, distance_to_glide_path: float) -> bool:
        reached_now = (
                distance_to_glide_path is not None and
                abs(distance_to_glide_path) < self.GLIDE_PATH_WIDTH_INCHES.value * 0.0254 * 0.5 and
                abs(degrees_left_to_rotate) < 4 * AimToDirectionConstants.ANGLE_TOLERANCE_DEGREES
        )
        if self._reached_glide_path_time and not reached_now:
            print(f"WARNING: not on glide path anymore (distance={distance_to_glide_path}, degrees={degrees_left_to_rotate}")

        return reached_now

    def update_vision(self, now):
        # non-sim logic:
        if self._camera.hasDetection():
            x = self._camera.getX()
            a = self._camera.getA()

            if x != 0 and a > 0:
                self._last_seed_distance_to_tag = self.compute_tag_distance_from_tag_size_on_frame(a)
                self._last_seen_object_time = now
                self._last_seed_object_size = a
                self._last_seed_object_x = x

                if not self._ever_saw_object:
                    self._ever_saw_object = True

        time_since_last_heartbeat = self._camera.getSecondsSinceLastHeartbeat()
        if time_since_last_heartbeat > self._frame_timeout:
            self._lost_tag = f"no camera heartbeat > {int(1000 * time_since_last_heartbeat)}ms"

        if self._last_seen_object_time != 0:
            time_since_last_detection = now - self._last_seen_object_time

            if time_since_last_detection > self._detection_timeout:
                if self._reached_final_approach_time == 0:
                    self._lost_tag = f"object lost for {int(1000 * time_since_last_detection)}ms before final approach"

                elif time_since_last_detection > self._detection_timeout + self._final_approach_seconds:
                    self._lost_tag = f"object lost for {int(1000 * time_since_last_detection)}ms on final approach"

    STATE_NAMES = [
        "starting",
        "catching glide path",
        "on glide path",
        "final approach",
        "finished",
        "lost tag",
    ]

    def get_state(self):
        if self._lost_tag:
            return 5
        if self._finished:
            return 4
        if self._reached_final_approach_time != 0:
            return 3
        if self._reached_glide_path_time != 0:
            return 2
        if self._last_seed_object_x != 1:
            return 1
        # otherwise, just starting
        return 0


class ApproachManually(Command):

    def __init__(
            self,
            camera,
            drivetrain,
            speed: Callable[[], float],
            specific_heading_degrees=None,
            reverse=False,
            settings: dict | None = None,
            camera_minimum_fps=4.0,
            dashboard_name="apmn"
    ):
        """
        Align the swerve robot to AprilTag precisely and then optionally slowly push it forward for a split second
        :param camera: camera to use, LimelightCamera or PhotonVisionCamera (from https://github.com/epanov1602/CommandRevSwerve/blob/main/docs/Adding_Camera.md)
        :param drivetrain: a drivetrain that implements swerve drive functionality (or mecanum/ball, must veer to sides)
        :param specific_heading_degrees: do you want the robot to face in a very specific direction? then specify it
        :param speed: function to get positive speed, even if camera is on the back of your robot (for the latter case set reverse=True)
        :param reverse: set it =True if the camera is on the back of the robot (not front)
        :param camera_minimum_fps: what is the minimal number of **detected** frames per second expected from this camera
        """
        super().__init__()
        assert hasattr(camera, "getX"), "camera must have `getX()` to give us the object coordinate (in degrees)"
        assert hasattr(camera, "getA"), "camera must have `getA()` to give us object size (in % of screen)"
        assert hasattr(camera, "getSecondsSinceLastHeartbeat"), "camera must have a `getSecondsSinceLastHeartbeat()`"
        assert hasattr(drivetrain, "drive"), "drivetrain must have a `drive()` function, because we need a swerve drive"

        self._drivetrain = drivetrain
        self.camera = camera
        self.addRequirements(drivetrain)
        self.addRequirements(camera)

        self.reverse = reverse
        self.speed = speed  # ensure that the speed is between 0.0 and 1.0
        if not callable(speed):
            self.speed = lambda: speed

        assert camera_minimum_fps > 0, f"non-positive cameraMinimumFps={camera_minimum_fps}"
        self.frameTimeoutSeconds = 1.0 / camera_minimum_fps

        # setting the target heading in a way that works for all cases
        self.targetDegrees = specific_heading_degrees
        if specific_heading_degrees is None:
            self.targetDegrees = lambda: self._drivetrain.heading.degrees()
        elif not callable(specific_heading_degrees):
            self.targetDegrees = lambda: specific_heading_degrees

        # state
        self.targetDirection = None
        self.lastSeenObjectTime = None
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = None
        self.everSawObject = False
        self.tReachedGlidePath = 0.0  # time when aligned to the tag and desired direction for the first time
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.xyReachedFinalApproach = Translation2d(0, 0)
        self.lostTag = ""
        self.finished = ""

        self.initTunables(settings, dashboard_name)

    def isReady(self, minRequiredObjectSize=0.3):
        return self.camera.hasDetection() and self.camera.getA() > minRequiredObjectSize

    def initTunables(self, settings, prefix):
        self.KPMULT_TRANSLATION = Tunable(settings, prefix, "GainTran", 0.6, (0.1, 8.0))  # gain for how quickly to move
        self.KPMULT_ROTATION = Tunable(settings, prefix, "GainRot", 0.8, (0.1, 8.0))  # gail for how quickly to rotate

        self.tunables = [
            self.KPMULT_TRANSLATION,
            self.KPMULT_ROTATION,
        ]

    def initialize(self):
        for t in self.tunables:
            t.fetch()

        print(f"ApproachTag: translation gain value {self.KPMULT_TRANSLATION.value}")

        self.targetDirection = Rotation2d.fromDegrees(self.targetDegrees())

        self.everSawObject = False
        self.lastSeenObjectX = 0.0
        self.lastSeenObjectSize = 0.0
        self.lastSeenDistanceToTag = 999
        self.lastSeenObjectTime = Timer.getFPGATimestamp()

        self.tReachedGlidePath = 0.0  # time when aligned to the tag and desired direction for the first time
        self.tReachedFinalApproach = 0.0  # time when reached the final approach
        self.xyReachedFinalApproach = Translation2d(0, 0)
        self.lostTag = ""
        self.finished = ""

        SmartDashboard.putString(f"command/{self.getName()}", "running")

    def isFinished(self) -> bool:
        return False  # never

    def end(self, interrupted: bool):
        self._drivetrain.stop()

    def execute(self):
        now = Timer.getFPGATimestamp()

        # 0. look at the camera
        self.update_vision(now)

        # 1. how many degrees are left to turn? (and recommended rotation speed)
        rotation_speed, degreesLeftToRotate = self.get_gyro_based_rotation_speed()

        # 2. how far from the glide path? (and recommended left translation speed)
        leftSpeed = self.get_vision_based_swerve_left_speed(now)

        # 3. the forward speed
        fwdSpeed = self.speed()
        fwdSpeed = fwdSpeed * abs(fwdSpeed)

        # 4. if we have not seen that object in a while (or about to lose it from sight), go slower
        if self.everSawObject:
            visionOld = (now - self.lastSeenObjectTime) / (0.5 * self.frameTimeoutSeconds)
            closeToEdge = abs(self.lastSeenObjectX) / 5.0 if self.lastSeenObjectX * rotation_speed > 0 else 0.0
            leftSpeed *= max(0.0, 1.0 - visionOld)
            rotation_speed *= max(0.25, 1.0 - closeToEdge)

        # 5. drive!
        if self.reverse:     # TODO: Make sure parameters are meters_per_second and radians_per_second
            self._drivetrain.drive(-fwdSpeed, -leftSpeed, rotation_speed, field_relative=False, rate_limit=True)
        else:
            self._drivetrain.drive(fwdSpeed, leftSpeed, rotation_speed, field_relative=False, rate_limit=True)

    def get_gyro_based_rotation_speed(self):
        # 1. how many degrees are left to turn?
        current_direction = self._drivetrain.heading
        rotation_remaining = self.targetDirection - current_direction
        degrees_remaining = rotation_remaining.degrees()

        # (optimize: do not turn left 350 degrees if you can just turn right -10 degrees, and vice versa)
        while degrees_remaining > 180:
            degrees_remaining -= 360
        while degrees_remaining < -180:
            degrees_remaining += 360

        # 2. proportional control: if we are almost finished turning, use slower turn speed (to avoid overshooting)
        proportional_speed = self.KPMULT_ROTATION.value * AimToDirectionConstants.kP * abs(degrees_remaining)
        if AimToDirectionConstants.USE_SQRT_CONTROL:
            proportional_speed = math.sqrt(0.5 * proportional_speed)  # will match the non-sqrt value when 50% max speed

        # 3. if target angle is on the right, we should really turn right (negative turn speed)
        turn_speed = min([proportional_speed, 1.0])
        if degrees_remaining < 0:
            turn_speed = -turn_speed

        return turn_speed, degrees_remaining

    def get_vision_based_swerve_left_speed(self, _now):
        # can we trust the last seen object?
        if not (self.lastSeenObjectSize > 0):
            return 0.0  # the object is not yet there, hoping that this is temporary

        # where are we?
        robot_x, robot_y = self.localize()

        # which speed to use to reduce robotY
        left_speed = self.compute_proportional_speed(abs(robot_y))

        if robot_y > 0:
            left_speed = -left_speed

        return left_speed

    def localize(self):
        """
        localize the robot camera in the frame of the tag
        """
        distance_to_tag = self.lastSeenDistanceToTag

        # trigonometry: how many meters on the left is our tag? (if negative, then it's on the right)
        angle = Rotation2d.fromDegrees(self.lastSeenObjectX)
        y = -distance_to_tag * angle.sin()

        distance_to_final_approach = distance_to_tag
        return -distance_to_final_approach, -y

    def compute_proportional_speed(self, distance) -> float:
        kp_mult_tran = self.KPMULT_TRANSLATION.value
        velocity = distance * GoToPointConstants.KP_TRANSLATE * kp_mult_tran

        if GoToPointConstants.USE_SQRT_CONTROL:
            velocity = math.sqrt(0.5 * velocity * kp_mult_tran)

        if velocity > 1.0:
            velocity = 1.0

        if velocity < GoToPointConstants.MIN_TRANSLATE_SPEED:
            velocity = GoToPointConstants.MIN_TRANSLATE_SPEED

        return velocity

    @staticmethod
    def compute_tag_distance_from_tag_size_on_frame(object_size: percent):
        """
        # if a 0.2*0.2 meter AprilTag appears to take 1% of the screen on a 1.33-square-radian FOV camera...
        #   angular_area = area / distance^2
        #   4 * 0.01 = 0.2 * 0.2 / distance^2
        #   distance = sqrt(0.2 * 0.2 / (1.33 * 0.03)) = 1.0 meters
        # ... then it must be 1.0 meters away!
        #
        # in other words, we can use this approximate formula for distance (if we have 0.2 * 0.2 meter AprilTag)
        """
        return math.sqrt(0.2 * 0.2 / (1.70 * 0.01 * object_size))
        # note: Arducam w OV9281 (and Limelight 3 / 4) is 0.57 sq radians (not 1.33)

    def update_vision(self, now):
        if self.camera.hasDetection():
            x = self.camera.getX()
            a = self.camera.getA()

            if x != 0 and 0 < a < 3:
                self.lastSeenDistanceToTag = self.compute_tag_distance_from_tag_size_on_frame(a)
                self.lastSeenObjectTime = now
                self.lastSeenObjectSize = a
                self.lastSeenObjectX = x

                if not self.everSawObject:
                    self.everSawObject = True
