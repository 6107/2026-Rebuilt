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

    from typing import List, Optional, Callable, Dict, Any

    from ntcore import NetworkTableInstance
    from robotpy_apriltag import AprilTagField, AprilTagFieldLayout
    from wpimath.geometry import Transform3d, Rotation2d, Pose3d, Pose2d, Rotation3d
    from wpimath.units import milliseconds, seconds, meters, degrees

    import constants
    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData
    from lib_6107.subsystems.vision.photonvision import PhotonVisionSubsystem
    from lib_6107.util.field import Field
    from lib_6107.subsystems.pykit.vision_io import VisionIO, TargetObservation, \
        PoseObservation, PoseObservationType

    from photonlibpy import PhotonCamera, PhotonPoseEstimator
    from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult, PhotonTrackedTarget, \
        MultiTargetPNPResult
    from photonlibpy.simulation import PhotonCameraSim, SimCameraProperties, VisionSystemSim

    logger = logging.getLogger(__name__)
    NIL_POSE_3D = Pose3d(0.0, 0.0, 0.0, Rotation3d(0.0, 0.0, 0.0))


    class PhotonVisionSubsystemSim(PhotonVisionSubsystem):
        """
        Simulation wrapper or the PhotonVision subsystem
        """

        def __init__(self, info: Dict[str, Any], drivetrain: 'DriveSubsystem', field: Field):
            super().__init__(info, drivetrain, field)

            self._vision_sim: VisionSystemSim = VisionSystemSim(self._name)

            # Add apriltags to the sim
            self._vision_sim.addAprilTags(field.layout)

            # Add the simulated camera
            properties: SimCameraProperties = SimCameraProperties()
            self._sim_camera = PhotonCameraSim(self._camera, properties, field.layout)

            # Finish initialization by adding the camera to the simulation
            self._vision_sim.addCamera(self._sim_camera, self._camera_transform)

        @property
        def _get_robot_pose(self) -> Pose2d | Pose3d:
            # Try to get a Pose3d. If that fails,drop back to Pose 2d
            # TODO: Better property name

            pose: Pose2d = self._drivetrain.pose
            pose3d = self._drivetrain.get_robot_3d(pose, Rotation3d.fromDegrees(self._drivetrain.gyro.inputs.roll,
                                                                                self._drivetrain.gyro.inputs.pitch,
                                                                                self._drivetrain.gyro.inputs.yaw))

            return pose3d if pose3d != NIL_POSE_3D else self._drivetrain.pose

        def updateInputs(self, inputs: VisionIO.VisionIOInputs) -> None:
            self._vision_sim.update(self._get_robot_pose)
            super().updateInputs(inputs)

except ImportError as _e:
    raise
