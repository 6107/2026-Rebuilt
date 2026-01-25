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

import logging
from typing import Optional, Tuple, Any

from commands2 import Subsystem
from robotpy_apriltag import AprilTagFieldLayout, AprilTagField
from wpilib import RobotBase
from wpimath.geometry import Transform3d
from wpimath.units import degrees, percent

import constants
from lib_6107.util.field import Field

logger = logging.getLogger(__name__)


class VisionTargetData:
    def __init__(self, yaw: degrees, pitch: degrees, area: percent,
                 fudicial_id: int, pose_ambiguity: float,
                 best_target_transform: Transform3d,
                 alternate_target_transform: Transform3d):
        # The yaw of the target in degrees (positive left)
        self.yaw: degrees = yaw

        # The pitch of the target in degrees (positive up)
        self.pitch: degrees = pitch

        # The area (how much of the camera feed the bounding box takes up) as a percent (0-100)
        self.area: percent = area

        # AprilTag related data (if any)
        # The ID of the detected fiducial marker
        self.april_tag_id: int = fudicial_id

        # How ambiguous the pose of the target is
        self.pose_ambiguity: float = pose_ambiguity

        # Get the transform that maps camera space (X = forward, Y = left, Z = up)
        # to object/fiducial tag space (X forward, Y left, Z up) with the lowest
        # reprojection error
        self.best_target_transform: Transform3d = best_target_transform

        # Get the transform that maps camera space (X = forward, Y = left, Z = up)
        # to object/fiducial tag space (X forward, Y left, Z up) with the highest
        # reprojection error
        self.alternate_target_transform: Transform3d = alternate_target_transform


class VisionSubsystem(Subsystem):
    def __init__(self, name: str, field: Field, transform: Transform3d, drivetrain: 'DriveSubsystem'):
        super().__init__()

        # Load the initial field layout. This can be changed later at
        # the beginning of the Autonomous or Teleop periods
        self._april_tag_field: Optional[AprilTagField] = field.field
        self._field_layout: Optional[AprilTagFieldLayout] = field.layout
        self._name = name

        self._camera_transform: Transform3d = transform
        self._drivetrain: 'DriveSubsystem' = drivetrain
        self._is_simulation: bool = RobotBase.isSimulation()

    @staticmethod
    def create(camera_type: str, name: str, field: Field, localizer: bool,
               transform: Transform3d, drivetrain: 'DriveSubsystem') -> Tuple[
        Optional['VisionSubsystem'], Optional['Subsystem']]:

        camera_subsystem: Optional['VisionSubsystem'] = None
        localizer_subsystem: Optional['Subsystem'] = None

        match camera_type:
            case constants.CAMERA_TYPE_LIMELIGHT:
                pass
                # self.localizer = LimelightLocalizer(self, self.robot_drive)
                # self.localizer.addCamera(self.camera,
                #                          cameraPoseOnRobot=pose["Pose"],
                #                          cameraHeadingOnRobot=pose["Heading"])

            case constants.CAMERA_TYPE_PHOTONVISION:
                try:
                    from lib_6107.subsystems.vision.photonvision import PhotonVisionSubsystem
                    camera_subsystem = PhotonVisionSubsystem(name, field, transform, drivetrain)

                    # if localizer:
                    #     localizer_subsystem = PhotonLocalizer(self, self.robot_drive, "2025-reefscape.json")
                except ImportError as e:
                    logger.error(f"PhotonVisionSubsystem not found, camera {name}: {e}")

        return camera_subsystem, localizer_subsystem

    @property
    def april_tag_field(self) -> AprilTagField:
        return self._april_tag_field

    def get_latest_results(self) -> Optional[Any]:
        raise NotImplementedError("Implement in subclass")

    def simulationPeriodic(self):
        pass  # For now  TODO: Can any of this be common
