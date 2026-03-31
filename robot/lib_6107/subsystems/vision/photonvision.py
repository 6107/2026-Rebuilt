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

    from typing import Dict, List, Any, Optional

    from ntcore import NetworkTableInstance
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
    from wpimath.geometry import Transform3d, Rotation2d, Pose3d
    from wpimath.units import milliseconds, seconds, meters, percent, degrees

    import constants
    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData, VisionConsumer
    from lib_6107.util.field import Field
    from lib_6107.subsystems.pykit.vision_io import VisionIO, TargetObservation, \
        PoseObservation, PoseObservationType
    from robot_2026.util.logtracer import LogTracer

    from photonlibpy import PhotonCamera, PhotonPoseEstimator
    from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult, PhotonTrackedTarget, \
        MultiTargetPNPResult

    logger = logging.getLogger(__name__)


    class PhotonVisionSubsystem(VisionSubsystem):
        def __init__(self, info: Dict[str, Any], drivetrain: 'DriveSubsystem', field: Field):
            super().__init__(info, drivetrain, field)

            self._camera: PhotonCamera = PhotonCamera(self._name)
            self._latest_results: Optional[PhotonPipelineResult] = None
            self._estimator: Optional[PhotonPoseEstimator] = None

            if self._estimate:
                # The estimator is used if the drive subsystem does not support a way to add
                # individual pose elements into it's pose estimation. For the CTRE (phoenix6)
                # drives, there is a supported method to add individual elements and so we do
                # not set up our own.
                self._estimator: PhotonPoseEstimator = PhotonPoseEstimator(self._field_layout,
                                                                           self._camera_transform)
                # Register for field layout changes
                field.register_layout_callback(self._on_field_change)

        def _on_field_change(self, _field: AprilTagField, layout: AprilTagFieldLayout) -> None:
            """
            Operator selected a different field layout.
            """
            if self._estimator is not None:
                self._estimator.fieldTags = layout

        def _get_latest_results(self) -> Optional[PhotonPipelineResult]:
            self._latest_results = self._camera.getLatestResult()
            return self._latest_results

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

            return VisionTargetData(photon_target.yaw, photon_target.pitch,
                                    photon_target.area, photon_target.fiducialId,
                                    photon_target.poseAmbiguity,
                                    photon_target.bestCameraToTarget,
                                    photon_target.altCameraToTarget)

        @property
        def valid(self) -> bool:
            raise self._camera is not None and self._camera.isConnected()

        @property
        def area(self) -> percent:
            """
            Target Area (0..100] percent of image
            """
            target: Optional[VisionTargetData] = self.best_target
            return target.area if target else 0

        @property
        def x_offset(self) -> degrees:
            """
            Horizontal Offset from Crosshair to Target
            """
            target: Optional[VisionTargetData] = self.best_target
            return target.yaw if target else 0

        @property
        def y_offset(self) -> degrees:
            """
            Vertical Offset from Crosshair to Target
            """
            target: Optional[VisionTargetData] = self.best_target
            return target.pitch if target else 0

        def get_latest_results(self) -> Optional[PhotonPipelineResult]:
            return self._latest_results

        def periodic(self) -> None:
            super().periodic()

            if not self._is_simulation:
                # Clear latest_results so we will get new results on the next pass
                self._latest_results = None

        def simulationPeriodic(self):
            """
            This method is called periodically by the CommandScheduler (after the periodic
            function). It is useful for updating subsystem-specific state that needs to be
            maintained for simulations, such as for updating simulation classes and setting
            simulated sensor readings.

            Unlike the physics 'update_sim', it is not called with the current time (now)
            or the amount of time since 'update_sim' was called (tm_diff).  It is called
            just after the 'periodic' call and before the 'update_sim' is called. One other
            'important' difference is 'update_sim' is called at a period >= 10 ms instead
            of the default 20 mS for the CommandScheduler's simulationPeriodic (this function).
            """
            LogTracer.resetOuter(f"{self.getName()}-simulationPeriodic")

            super().simulationPeriodic()

            # Update simulation based on physics engine (e.g., swerve drive sim)
            sim_robot_pose = self._drivetrain.pose

            # TODO: PhotonVision has quite a few things to support simulation...
            # Simulate camera seeing tags based on current pose
            # (This requires using VisionSystemSim in more complex setups)
            # ...

            # Update estimator with simulated data
            # if self._estimator is not None:
            # TODO: self._estimator.update()

            # Clear latest_results so we will get new results on the next pass
            self._latest_results = None
            LogTracer.recordTotal()

        def update_sim(self, now: float, tm_diff: float) -> None:
            """
            Called when the simulation parameters for the program need to be updated.
            This function is called from the '_simulationPeriodic' function of the
            robotpy core routine and is called at a period >= 10 mS. Note that the
            CommandScheduler also has an 'simulationPeriodic' function that it calls
            into all Command2 based subsystems at its update period which has a
            default rate of 20 mS.

            This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
            that function uses pykit's logging method, you should use those values in
            your simulation.

            :param now:     The current time as a float
            :param tm_diff: The amount of time that has passed since the last
                            time that this function was called
            """
            super().update_sim(now, tm_diff)

        def updateInputs(self, inputs: VisionIO.VisionIOInputs) -> None:
            """
            Pykit support for AdvantageScope.  Called from base class's 'periodic' function
            """
            inputs.connected = self._camera.isConnected()

            # Read new camera observations
            # TODO: In java version of this, this is a set. We want a set but we need to keep order perhaps?
            inputs.tag_ids = []
            inputs.pose_observations = []

            for result in self._camera.getAllUnreadResults():
                # Update latest target observation

                if result is not None and result.hasTargets():
                    inputs.latest_target_observation = \
                        TargetObservation(Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                                          Rotation2d.fromDegrees(result.getBestTarget().getPitch()))
                else:
                    inputs.latest_target_observation = TargetObservation(Rotation2d(0),
                                                                         Rotation2d(0))
                # Add pose observation
                multi_tag_result: MultiTargetPNPResult = result.multitagResult
                single_target: PhotonTrackedTarget = result.getBestTarget()

                if multi_tag_result is not None:
                    # Multi-tag result processing
                    # Calculate robot pose
                    field_to_camera: Transform3d = multi_tag_result.estimatedPose.best
                    field_to_robot: Transform3d = field_to_camera + self._camera_transform.inverse()
                    robot_pose: Pose3d = Pose3d(field_to_robot.translation(), field_to_robot.rotation())

                    # Calculate average tag distance
                    total_tag_distance: meters = sum(target.bestCameraToTarget.translation().norm()
                                                     for target in result.targets)
                    # Add tag IDs
                    inputs.tag_ids = multi_tag_result.fiducialIDsUsed

                    # Add observation
                    inputs.pose_observations.append(
                        PoseObservation(result.getTimestampSeconds(),
                                        robot_pose,
                                        multi_tag_result.estimatedPose.ambiguity,
                                        len(multi_tag_result.fiducialIDsUsed),
                                        total_tag_distance / len(result.targets),
                                        PoseObservationType.PHOTONVISION))

                elif single_target is not None:
                    # Single target acquired. Note this is also the 'best' if it was multi-target but
                    # that is handled in the previous 'if' clause
                    #
                    # Calculate robot pose

                    tag_pose = self._field_layout.getTagPose(single_target.fiducialId)
                    if tag_pose:
                        field_to_target: Transform3d = Transform3d(tag_pose.translation(), tag_pose.rotation())
                        camera_to_target: Transform3d = single_target.bestCameraToTarget
                        field_to_camera: Transform3d = field_to_target + camera_to_target.inverse()
                        field_to_robot: Transform3d = field_to_camera + self._camera_transform.inverse()

                        robot_pose: Pose3d = Pose3d(field_to_robot.translation(), field_to_robot.rotation())

                        # Add tag ID
                        inputs.tag_ids = [single_target.fiducialId]

                        # Add observation
                        inputs.pose_observations = [PoseObservation(result.getTimestampSeconds(),
                                                                    robot_pose,  # 3D pose estimate
                                                                    single_target.poseAmbiguity,  # Ambiguity
                                                                    1,  # Tag count
                                                                    camera_to_target.translation().norm(),
                                                                    # Average tag distance
                                                                    PoseObservationType.PHOTONVISION)]

except ImportError as _e:
    raise
