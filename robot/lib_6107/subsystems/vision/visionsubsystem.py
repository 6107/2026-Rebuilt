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
from typing import Any, Dict, Optional, Tuple

from commands2 import Subsystem
from ntcore import NetworkTable, NetworkTableInstance
from robotpy_apriltag import AprilTagDetector, AprilTagField, AprilTagFieldLayout
from wpilib import RobotBase, SmartDashboard
from wpimath.geometry import Rotation2d, Transform3d
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
    def __init__(self, camera_name: str, field: Field, transform: Transform3d, drivetrain: 'DriveSubsystem'):
        super().__init__()

        # Load the initial field layout. This can be changed later at
        # the beginning of the Autonomous or Teleop periods
        self._april_tag_field: Optional[AprilTagField] = field.field
        self._field_layout: Optional[AprilTagFieldLayout] = field.layout
        self._name = camera_name
        self._robot = drivetrain.robot

        self._camera_transform: Transform3d = transform
        self._drivetrain: 'DriveSubsystem' = drivetrain
        self._is_simulation: bool = RobotBase.isSimulation()

        # April tag setup
        self._tag_detector = AprilTagDetector()
        self._tag_detector.addFamily("tag16h5")

        # NetworkTable setup
        nt_instance = NetworkTableInstance.getDefault()
        self._network_table: NetworkTable = nt_instance.getTable(camera_name)

    @staticmethod
    def create(info: Dict[str, Any], field: Field,
               drivetrain: 'DriveSubsystem') -> Tuple[Optional['VisionSubsystem'], Optional['Subsystem']]:

        camera_type = info.get("Type", constants.CAMERA_TYPE_NONE)
        localizer = info.get("Localizer")
        transform: Transform3d = info.get("Pose")
        camera_name = info.get("Name", camera_type)
        heading = info.get("Heading", Rotation2d.fromDegrees(0))

        camera_subsystem: Optional['VisionSubsystem'] = None
        localizer_subsystem: Optional['Subsystem'] = None

        match camera_type:
            case constants.CAMERA_TYPE_LIMELIGHT:
                from lib_6107.subsystems.vision.limelightvision import LimelightVisionSubsystem
                camera_subsystem = LimelightVisionSubsystem(camera_name, field, transform, drivetrain)
                # self.localizer = LimelightLocalizer(self, self.robot_drive)
                # self.localizer.addCamera(self.camera,
                #                          cameraPoseOnRobot=pose["Pose"],
                #                          cameraHeadingOnRobot=pose["Heading"])

            case constants.CAMERA_TYPE_PHOTONVISION:
                try:
                    from lib_6107.subsystems.vision.photonvision import PhotonVisionSubsystem
                    camera_subsystem = PhotonVisionSubsystem(camera_name, field, transform, drivetrain)

                    # if localizer:
                    #     pass # TODO: Need to support
                    #     localizer_subsystem = PhotonLocalizer(self, self.robot_drive, "2025-reefscape.json")
                except ImportError as e:
                    logger.error(f"PhotonVisionSubsystem not found, camera {camera_name}: {e}")

        return camera_subsystem, localizer_subsystem

    @property
    def name(self) -> str:
        return self._name

    @property
    def april_tag_field(self) -> AprilTagField:
        return self._april_tag_field

    def get_latest_results(self) -> Optional[Any]:
        raise NotImplementedError("Implement in subclass")

    def periodic(self):

        pass  # For now  TODO: Can any of this be common

    def simulationPeriodic(self):
        pass  # For now  TODO: Can any of this be common

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        # SmartDashboard.putData("Field", self.field)
        SmartDashboard.putString('Camera/name', self._name)
        # SmartDashboard.putString('Camera/type', "Limelight")

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        # if update_dash:
        #     SmartDashboard.putString('Camera/heartbeat', "Alive" if self.heartbeating else "Dead")
        #     SmartDashboard.putNumber('Camera/last-heartbeat', self.lastHeartbeatTime)
