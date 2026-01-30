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

try:
    import logging

    from typing import Optional

    from wpilib import Timer
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
    from wpimath.geometry import Transform3d
    from wpimath.units import milliseconds, seconds

    from ntcore import DoubleArrayTopic, DoubleTopic, DoubleEntry, IntegerTopic, \
        IntegerEntry, DoubleArrayPublisher, DoublePublisher, DoubleArrayEntry, \
        IntegerPublisher

    from limelight import Limelight
    # from limelightresults import FiducialResult, GeneralResult, DetectorResult
    # from lib_6107.subsystems.vision.deprecated.limelight_camera import LimelightCamera
    # from lib_6107.subsystems.vision.deprecated.limelight_localizer import LimelightLocalizer

    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData
    from lib_6107.util.field import Field

    logger = logging.getLogger(__name__)


    class LimelightVisionSubsystem(VisionSubsystem):
        def __init__(self, name: str, field: Field, transform: Transform3d, drivetrain: 'DriveSubsystem'):
            super().__init__(name, field, transform, drivetrain)

            self._camera: Limelight = Limelight(name)

            self._path: str = self._network_table.getPath()

            # self._latest_results: Optional[LimelightResults] = None

            self._pipeline_index_request: DoublePublisher = self._network_table.getDoubleTopic("pipeline").publish()
            self._pipeline_index: DoubleEntry = self._network_table.getDoubleTopic("getpipe").getEntry(-1)
            # "cl" and "tl" are additional latencies in milliseconds

            self._led_mode: IntegerEntry = self._network_table.getIntegerTopic("ledMode").getEntry(-1)
            self._cam_mode: IntegerEntry = self._network_table.getIntegerTopic("camMode").getEntry(-1)
            self._tx: DoubleEntry = self._network_table.getDoubleTopic("tx").getEntry(0.0)
            self._ty: DoubleEntry = self._network_table.getDoubleTopic("ty").getEntry(0.0)
            self._ta: DoubleEntry = self._network_table.getDoubleTopic("ta").getEntry(0.0)
            self._hb: IntegerEntry = self._network_table.getIntegerTopic("hb").getEntry(0)

            self._last_heartbeat = 0
            self._last_heartbeat_time = 0
            self._heart_beating = False

            self._localizer_subscribed = False
            self._robot_orientation_set_request: Optional[DoubleArrayPublisher] = None
            self._camera_pose_set_request: Optional[DoubleArrayPublisher] = None
            self._imu_mode_request: Optional[IntegerPublisher] = None  # this is only for Limelight 4

            # and we can then receive the localizer results from the camera back
            self._robot_pose: DoubleArrayEntry = self._network_table.getDoubleArrayTopic(
                "botpose_orb_wpiblue").getEntry([])
            self._robot_pose_flipped: DoubleArrayEntry = self._network_table.getDoubleArrayTopic(
                "botpose_orb_wpired").getEntry([])

            # Load the initial field layout
            # TODO: Is there a way to query existing field layout so we can possibly skip this
            #       step?
            # TODO: Below needs JSON
            #  self._camera.upload_fieldmap(self._field_layout)

        def _on_field_change(self, _field: AprilTagField, layout: AprilTagFieldLayout) -> None:
            """
            Operator selected a different field layout.
            """
            pass  # TODO: Support in future

        def add_localizer(self):
            if self._localizer_subscribed:
                return

            self._localizer_subscribed = True
            # if we want MegaTag2 localizer to work, we need to be publishing two things (to the camera):
            #   1. what robot's yaw is ("yaw=0 degrees" means "facing North", "yaw=90 degrees" means "facing West", etc.)
            #   2. where is this camera sitting on the robot (e.g. y=-0.2 meters to the right, x=0.1 meters fwd from center)
            self._robot_orientation_set_request = self._network_table.getDoubleArrayTopic(
                "robot_orientation_set").publish()
            self._camera_pose_set_request = self._network_table.getDoubleArrayTopic(
                "camerapose_robotspace_set").publish()
            self._imu_mode_request = self._network_table.getIntegerTopic(
                "imumode_set").publish()  # this is only for Limelight 4

            # and we can then receive the localizer results from the camera back
            self._robot_pose = self._network_table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
            self._robot_pose_flipped = self._network_table.getDoubleArrayTopic("botpose_orb_wpired").getEntry([])

        @property
        def pipeline(self) -> int:
            return int(self._pipeline_index.get(-1))

        @pipeline.setter
        def pipeline(self, index: int) -> None:
            self._pipeline_index_request.set(float(index))

        @property
        def latency(self) -> Optional[milliseconds]:
            return None

        @property
        def timestamp(self) -> Optional[seconds]:
            """
            Returns the estimated time the frame was taken, in the Received system's time base
            """
            return None

        @property
        def best_target(self) -> Optional[VisionTargetData]:
            """
            Returns the best target in this pipeline result. If there are no targets, this method will
            return null. The best target is determined by the target sort mode in the PhotonVision UI.
            """
            return None

        @property
        def a(self) -> float:
            return self._ta.get()

        @property
        def x(self) -> float:
            return self._tx.get()

        @property
        def y(self) -> float:
            return self._ty.get()

        @property
        def hb(self) -> float:
            return self._hb.get()

        def detection(self) -> bool:
            return self.x != 0.0 and self._heart_beating

        def get_seconds_since_last_heartbeat(self) -> float:
            return Timer.getFPGATimestamp() - self._last_heartbeat_time

        def periodic(self) -> None:
            super().periodic()

            now = Timer.getFPGATimestamp()
            heartbeat = self.hb

            if heartbeat != self._last_heartbeat:
                self._last_heartbeat = heartbeat
                self._last_heartbeat_time = now

            heart_beating = now < self._last_heartbeat_time + 5  # no heartbeat for 5s => stale camera

            if heart_beating != self._heart_beating:
                logger.warning(f"Camera {self._name}: {'UPDATING' if heart_beating else 'NO LONGER UPDATING'}")

            self._heart_beating = heart_beating

            # self.dashboard_periodic()

        def simulationPeriodic(self):
            super().simulationPeriodic()

            pass


except ImportError as _e:
    pass
