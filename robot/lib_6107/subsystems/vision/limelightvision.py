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

    from typing import List, Optional, Dict, Any

    from wpilib import Timer, RobotController
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
    from wpimath.geometry import Transform3d, Rotation2d, Pose3d, Rotation3d
    from wpimath.units import milliseconds, seconds, degrees, percent, degreesToRadians

    from ntcore import DoubleArrayTopic, DoubleTopic, DoubleEntry, IntegerTopic, \
        IntegerEntry, DoubleArrayPublisher, DoublePublisher, DoubleArrayEntry, \
        IntegerPublisher, DoubleSubscriber, DoubleArraySubscriber

    from limelight import Limelight
    # from limelightresults import FiducialResult, GeneralResult, DetectorResult
    # from lib_6107.subsystems.vision.deprecated.limelight_camera import LimelightCamera
    # from lib_6107.subsystems.vision.deprecated.limelight_localizer import LimelightLocalizer

    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData, VisionConsumer
    from lib_6107.util.field import Field
    from lib_6107.subsystems.pykit.vision_io import VisionIO, TargetObservation, \
        PoseObservation, PoseObservationType

    logger = logging.getLogger(__name__)


    class LimelightVisionSubsystem(VisionSubsystem):
        def __init__(self, info: Dict[str, Any], drivetrain: 'DriveSubsystem', field: Field):
            super().__init__(info, drivetrain, field)

            self._camera: Limelight = Limelight(self._name)

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

            # TODO: New pykit required changes
            # TODO: Some of the subscribers or publishers may be duplicated with 'add_localizer' code that is pre-pykit (and never tested)
            # I/O Implementation for real Limelight hardware
            # self.rotationSupplier: Supplier<Rotation2d>     = None
            self.orientation_publisher: DoubleArrayPublisher = self._network_table.getDoubleArrayTopic(
                "robot_orientation_set").publish()

            self.latency_subscriber: DoubleSubscriber = self._network_table.getDoubleTopic("tl").subscribe(0.0)
            self.tx_subscriber: DoubleSubscriber = self._network_table.getDoubleTopic("tx").subscribe(0.0)
            self.ty_subscriber: DoubleSubscriber = self._network_table.getDoubleTopic("ty").subscribe(0.0)
            self.megatag1_subscriber: DoubleArraySubscriber = self._network_table.getDoubleArrayTopic(
                "botpose_wpiblue").subscribe([])
            self.megatag2_subscriber: DoubleArraySubscriber = self._network_table.getDoubleArrayTopic(
                "botpose_orb_wpiblue").subscribe([])

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
        def valid(self) -> bool:
            return self.x != 0.0 and self._heart_beating

        @property
        def area(self) -> percent:
            """
            Target Area (0..100] percent of image
            """
            return self._ta.get()

        @property
        def x_offset(self) -> degrees:
            """
            Horizontal Offset from Crosshair to Target [-29.9..29.8] degrees
            """
            return self._tx.get()

        @property
        def y_offset(self) -> degrees:
            """
            Vertical Offset from Crosshair to Target [-24.85..24.85]
            """
            return self._ty.get()

        @property
        def hb(self) -> float:
            """
            Heartbeat value. Increases once per frame and rolls over at 2 billion.
            """
            return self._hb.get()

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

            # Update SmartDashboard for this subsystem at a rate slower than the period
            counter = self._robot.counter
            if counter % 100 == 0 or (self._robot.counter % 15 == 0 and
                                      self._robot.isEnabled()):
                self.dashboard_periodic()

        def simulationPeriodic(self):
            super().simulationPeriodic()

            pass

        def updateInputs(self, inputs: VisionIO.VisionIOInputs) -> None:
            """
            Pykit support for AdvantageScope
            """
            # TODO: Tie in the heartbeat with the determination of 'connected' below
            inputs.connected = (RobotController.getFPGATime() - self.latency_subscriber.getLastChange()) / 1000 < 250

            inputs.latest_target_observation = TargetObservation(Rotation2d.fromDegrees(self.tx_subscriber.get()),
                                                                 Rotation2d.fromDegrees(self.ty_subscriber.get()), )
            # Update orientation for MegaTag 2
            # TODO: self.orientationPublisher.accept({rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0});
            # TODO: NetworkTableInstance.getDefault().flush(); // Increases network traffic but recommended by Limelight

            # Read new pose observations from NetworkTables
            # TODO: In java version of this, this is a set. We want a set but we need to keep order perhaps?
            inputs.tag_ids = []

            pose_observations: List[PoseObservation] = []
            for rawSample in self.megatag1_subscriber.readQueue():
                sample_len = len(rawSample.value)
                if sample_len == 0:
                    continue

                # TODO: Where in the heck does 11 come from and the increment by 7
                for i in range(11, sample_len, 7):
                    tag_id = rawSample.value[i]

                    if tag_id not in inputs.tag_ids:
                        inputs.tag_ids.append(int(tag_id))  # TODO: Is this an int always

                    # Pose contents:
                    # - Timestamp, based on server timestamp of publish and latency
                    # - 3D pose estimate
                    # - Ambiguity, using only the first tag because ambiguity isn't applicable for multitag
                    # - Tag Count
                    # - Average tag distance
                    # - Observation type
                    #
                    inputs.pose_observations.append(PoseObservation(
                        rawSample.time * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                        self.parse_pose(rawSample.value),
                        rawSample.value[17] if sample_len >= 18 else 0.0,
                        int(rawSample.value[7]),
                        rawSample.value[9],
                        PoseObservationType.MEGATAG_1))

            for rawSample in self.megatag2_subscriber.readQueue():
                sample_len = len(rawSample.value)
                if sample_len == 0:
                    continue

                # TODO: Where in the heck does 11 come from and the increment by 7
                for i in range(11, sample_len, 7):
                    tag_id = rawSample.value[i]

                    if tag_id not in inputs.tag_ids:
                        inputs.tag_ids.append(int(tag_id))  # TODO: Is this an int always

                    inputs.pose_observations.append(PoseObservation(
                        rawSample.time * 1.0e-6 - rawSample.value[6] * 1.0e-3,
                        self.parse_pose(rawSample.value),
                        0.0,
                        int(rawSample.value[7]),
                        rawSample.value[9],
                        PoseObservationType.MEGATAG_2))

        @staticmethod
        def parse_pose(raw_limlight_array: List[float]) -> Pose3d:
            """
            Parses the 3D pose from a Limelight botpose array
            """
            return Pose3d(raw_limlight_array[0], raw_limlight_array[1], raw_limlight_array[2],
                          Rotation3d(degreesToRadians(raw_limlight_array[3]),
                                     degreesToRadians(raw_limlight_array[4]),
                                     degreesToRadians(raw_limlight_array[5])))
except ImportError as _e:
    pass
