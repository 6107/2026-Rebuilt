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

import time

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import meters

from subsystems.swervedrive.drivesubsystem import DriveSubsystem


class LimeLightCtreLocalizer(Subsystem):
    def __init__(self, camera: 'VisionSubsystem', drive: DriveSubsystem):
        super().__init__()

        self._camera = camera
        self._drive: DriveSubsystem = drive
        self._instance: NetworkTableInstance = NetworkTableInstance.getDefault()
        self._table: NetworkTable = self._instance.getTable(camera.name)
        self._last_vision_add = time.monotonic()

        # Estimator not needed for CTRE drive subsystem. It has its own.
        # self._estimator = SwerveDrive4PoseEstimator(drive.kinematics,
        #                                             drive.gyro.get_rotation2d(),
        #                                             drive.get_module_positions(),
        #                                             Pose2d())

    def robotPeriodic(self):
        """
        Update the robot pose based on vision estimates
        """
        # Get the current pose from the drive subsystem. If vision is too far off, do not add the
        # measurement in.

        current_pose: Pose2d = self._drive.pose

        # Get Pose from Limelight (AprilTag)
        # botpose_wpiblue returns: [x, y, z, roll, pitch, yaw, latency]
        robot_pose = self._table.getNumberArray("botpose_wpiblue", [-1, -1, 0, 0, 0, 0, 0])
        timestamp: float = self._table.getEntry("ts").getDouble(0)
        x, y, latency = robot_pose[0], robot_pose[1], robot_pose[6]

        # TODO: Move next two to constants once we get this working
        MINIMUM_VISION_POSE_DELTA: meters = 1.0
        VISION_ADD_TIMEOUT = 2.0  # Two seconds

        # TODO: Add some smarts to the vision measurement standard deviation. Currently
        #       set to a high level of confidence since autonomous mode is dependent
        #       upon it.  When we enter teleop, we may want to adjust based on
        #
        #         o Number of april tags seen
        #         o If we are about to, or just have gone over a bump
        #         o If it is the beginning couple of seconds of teleop  (since we may be climbing down from ladder)
        #         o If we collide significantly into another object (perhaps from pose data).
        #
        #  Once we have several cameras, also need to figure out which one may have the best data.
        #
        if x >= 0 and y >= 0 and latency > 0 and timestamp > 0:
            # Well, did not get defaults back, how about distance
            now = time.monotonic()
            vision_timeout = (now - self._last_vision_add) >= VISION_ADD_TIMEOUT

            if vision_timeout or (abs(current_pose.x - x) < MINIMUM_VISION_POSE_DELTA and
                                  abs(current_pose.y - y) < MINIMUM_VISION_POSE_DELTA):
                # Add Vision Measurement to Odometry
                vision_pose = Pose2d(Translation2d(x, y), Rotation2d.fromDegrees(robot_pose[5]))

                # Using NT4 timestamp to handle latency
                timestamp = timestamp / 1e6 - (latency / 1000.0)
                self._drive.add_vision_measurement(vision_pose, timestamp)
                self._last_vision_add = now
