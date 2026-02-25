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

from dataclasses import dataclass
from enum import IntEnum, unique
from typing import List

from pykit.autolog import autolog
from wpimath.geometry import Pose3d, Rotation2d
from wpimath.units import meters, seconds


# from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData
# from lib_6107.util.field import Field
#
# from photonlibpy import PhotonCamera, PhotonPoseEstimator
# from photonlibpy.targeting.photonPipelineResult import PhotonPipelineResult, PhotonTrackedTarget

class TargetObservation:
    """
    Represents the angle to a simple target, not used for pose estimation
    """

    def __init__(self, tx: Rotation2d, ty: Rotation2d):
        self.tx: Rotation2d = tx
        self.ty: Rotation2d = ty


@unique
class PoseObservationType(IntEnum):
    NONE = 0
    MEGATAG_1 = 1
    MEGATAG_2 = 2
    PHOTONVISION = 3


class PoseObservation:
    """
    Represents a robot pose sample used for pose estimation
    """

    def __init__(self, timestamp: seconds, pose: Pose3d, ambiguity: float,
                 tag_count: int, avg_tag_distance: meters,
                 observation_type: PoseObservationType):
        self.timestamp: seconds = timestamp
        self.pose: Pose3d = pose
        self.ambiguity: float = ambiguity
        self.avg_tag_distance: meters = avg_tag_distance
        self.tag_count: int = tag_count
        self.observation_type: PoseObservationType = observation_type


class VisionIO:
    @autolog
    @dataclass
    class VisionIOInputs:
        """
        Loggable inputs for a Vision Subsystem sensor.
        """
        connected: bool = False
        latest_target_observation: TargetObservation = TargetObservation(Rotation2d(0), Rotation2d(0))
        pose_observations: List[PoseObservation] = None
        tag_ids: List[int] = None

    def updateInputs(self, inputs: VisionIOInputs) -> None:
        pass
