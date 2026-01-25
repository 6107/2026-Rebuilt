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

    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
    from wpimath.geometry import Transform3d
    from wpimath.units import milliseconds, seconds

    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData
    from lib_6107.util.field import Field

    from photonlibpy import PhotonCamera, PhotonPoseEstimator
    from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult, PhotonTrackedTarget

    logger = logging.getLogger(__name__)


    class PhotonVisionSubsystem(VisionSubsystem):
        def __init__(self, name: str, field: Field, transform: Transform3d, drivetrain: 'DriveSubsystem'):
            super().__init__(name, field, transform, drivetrain)

            self._camera: PhotonCamera = PhotonCamera(name)
            self._latest_results: Optional[PhotonPipelineResult] = None

            # self._sim_camera = TODO: More work her

            # In simulation, load the field layout
            self._estimator: PhotonPoseEstimator = PhotonPoseEstimator(self._field_layout,
                                                                       self._camera_transform)
            # Register for field layout changes
            field.register_layout_callback(self._on_field_change)

        def _on_field_change(self, field: AprilTagField, layout: AprilTagFieldLayout) -> None:
            logger.error(
                f"{self._name}: PhotonVisionSubsystem._on_field_change: not yet implemented: {field}, {layout}")

        @property
        def latency(self) -> Optional[milliseconds]:
            results = self._latest_results or self._get_latest_results()

            return results.getLatencyMillis() if results is not None else None

        @property
        def timestamp(self) -> Optional[seconds]:
            """
            Returns the estimated time the frame was taken, in the Received system's time base
            """
            results = self._latest_results or self._get_latest_results()

            return results.getTimestampSeconds() if results is not None else None

        @property
        def best_target(self) -> Optional[VisionTargetData]:
            """
            Returns the best target in this pipeline result. If there are no targets, this method will
            return null. The best target is determined by the target sort mode in the PhotonVision UI.
            """
            results = self._latest_results or self._get_latest_results()

            photon_target: PhotonTrackedTarget = results.getBestTarget() if results is not None else None
            if photon_target is None:
                return None

            return VisionTargetData(photon_target.yaw, photon_target.pitch, photon_target.area,
                                    photon_target.fiducialId, photon_target.poseAmbiguity,
                                    photon_target.bestCameraToTarget, photon_target.altCameraToTarget)

        def _get_latest_results(self) -> Optional[PhotonPipelineResult]:
            self._latest_results = self._camera.getLatestResult()
            return self._latest_results

        def periodic(self) -> None:
            if not self._is_simulation and self._estimator is not None:
                latest_result: Optional[PhotonPipelineResult] = self.get_latest_results()

                if latest_result is not None and latest_result.hasTargets:
                    self._estimator.TODO("WHAT TO DO HERE?")

                # Clear latest_results so we will get new results on the next pass
                self._latest_results = None

        def simulationPeriodic(self):
            # Update simulation based on physics engine (e.g., swerve drive sim)
            sim_robot_pose = self._drivetrain.pose

            # TODO: PhotonVision has quite a few things to support simulation...
            # Simulate camera seeing tags based on current pose
            # (This requires using VisionSystemSim in more complex setups)
            # ...

            # Update estimator with simulated data
            # TODO: elf._estimator.update()

            # Clear latest_results so we will get new results on the next pass
            self._latest_results = None

except ImportError:
    raise
