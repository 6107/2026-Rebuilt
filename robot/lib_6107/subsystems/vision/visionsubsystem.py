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
import math
from typing import Any, Callable, Dict, List, Optional

from commands2 import Subsystem
# noinspection PyPackageRequirements
from ntcore import NetworkTable, NetworkTableInstance
from robotpy_apriltag import AprilTagDetector, AprilTagField, AprilTagFieldLayout
from wpilib import Alert, RobotBase, SmartDashboard
from wpimath.geometry import Pose2d, Pose3d, Transform3d
from wpimath.units import degrees, milliseconds, percent, seconds

import constants
# noinspection PyPackageRequirements
from lib_6107.pykit.logger import Logger
from lib_6107.subsystems.pykit.vision_io import PoseObservation, PoseObservationType, VisionIO
from lib_6107.util.field import Field

logger = logging.getLogger(__name__)

VisionConsumer = Callable[[Pose2d, seconds, tuple[float,float,float] | None], None]

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


class VisionSubsystem(Subsystem, VisionIO):
    """
    Vision Subsystem

    Currently only support a single camera per vision subsystem. For the PhotonVision, we
    'could' eventually support multiple cameras from a single PhotonVision unit.

    TODO: Add multi-camera per VisionSubsystem support. Along these lines, we could just have a single
          vision subsystem, but have it run any combination of cameras...
    """

    def __init__(self, info: Dict[str, Any], drivetrain: 'DriveSubsystem', field: Field):
        Subsystem.__init__(self)
        VisionIO().__init__()

        # Load the initial field layout. This can be changed later at
        # the beginning of the Autonomous or Teleop periods
        self._april_tag_field: Optional[AprilTagField] = field.field
        self._field_layout: Optional[AprilTagFieldLayout] = field.layout
        self._name = info.get("Name", info.get("Type"))
        self._robot = drivetrain.robot
        self._estimate = info.get("Localizer", False)
        self._std_dev_factor = info.get("Trust", 0.1)
        self._camera_transform: Transform3d = info.get("Transform")
        self._drivetrain: 'DriveSubsystem' = drivetrain
        self._is_simulation: bool = RobotBase.isSimulation()
        # self._vision_input = vision_input TODO: Pass in 'AddVisionMeasurement' callable here?
        self._inputs = VisionIO.VisionIOInputs()

        # April tag setup
        self._tag_detector = AprilTagDetector()
        self._tag_detector.addFamily("tag16h5")

        # NetworkTable setup
        nt_instance = NetworkTableInstance.getDefault()
        self._network_table: NetworkTable = nt_instance.getTable(self._name)

        # TODO: Also look at AdvantageKit java 'Vision.java' for additional work
        #       needed here.
        # Initialize disconnected alerts
        self._disconnected_alert = Alert(f"Vision camera {self.name}", Alert.AlertType.kWarning)

    @staticmethod
    def create(info: Dict[str, Any], drivetrain: 'DriveSubsystem', field: Field) -> VisionSubsystem | None:

        camera_type = info.get("Type", constants.CAMERA_TYPE_NONE)
        camera_subsystem: Optional[VisionSubsystem] = None

        match constants.ROBOT_MODE:
            case constants.RobotModes.REAL:
                pass  # TODO: In the 'match' below, use these values to create a real or simulated camera

            case constants.RobotModes.SIMULATION:
                pass  # TODO: In the 'match' below, use these values to create a real or simulated camera

            case constants.RobotModes.REPLAY:
                pass  # TODO: In the 'match' below, use this to create a simulated replay camera

        # TODO: For items above, look at the RobotContainer.java from the vision template for AdvantageKit

        match camera_type:
            case constants.CAMERA_TYPE_LIMELIGHT:
                # TODO: For limelight, allow multiple cameras to be specified
                from lib_6107.subsystems.vision.limelightvision import LimelightVisionSubsystem
                camera_subsystem = LimelightVisionSubsystem(info, drivetrain, field)

            case constants.CAMERA_TYPE_PHOTONVISION:
                try:
                    match constants.ROBOT_MODE:
                        case constants.RobotModes.REAL:
                            from lib_6107.subsystems.vision.photonvision import PhotonVisionSubsystem
                            camera_subsystem = PhotonVisionSubsystem(info, drivetrain, field)

                        case constants.RobotModes.SIMULATION:
                            from lib_6107.subsystems.vision.photonvision_sim import PhotonVisionSubsystemSim
                            camera_subsystem = PhotonVisionSubsystemSim(info, drivetrain, field)

                        case constants.RobotModes.REPLAY:
                            from lib_6107.subsystems.vision.photonvision_sim import PhotonVisionSubsystemSim
                            camera_subsystem = PhotonVisionSubsystemSim(info, drivetrain, field)

                except ImportError as e:
                    logger.error(f"PhotonVisionSubsystem not found, camera {info.get("Name", "n/a")}: {e}")

        return camera_subsystem

    @property
    def name(self) -> str:
        return self._name

    @property
    def inputs(self) -> VisionIO.VisionIOInputs:
        return self._inputs

    @property
    def drivetrain(self) -> 'DriveSubsystem':
        return self._drivetrain

    @property
    def april_tag_field(self) -> AprilTagField:
        return self._april_tag_field

    @property
    def pipeline(self) -> int:
        raise NotImplementedError("pipeline-get: Implement in subclass")

    @pipeline.setter
    def pipeline(self, index: int) -> None:
        raise NotImplementedError("pipeline-set: Implement in subclass")

    @property
    def latency(self) -> Optional[milliseconds]:
        return None

    @property
    def timestamp(self) -> Optional[seconds]:
        """
        Returns the estimated time the frame was taken, in the Received system's time base
        """
        raise NotImplementedError("timestamp: Implement in subclass")

    @property
    def best_target(self) -> Optional[VisionTargetData]:
        """
        Returns the best target in this pipeline result. If there are no targets, this method will
        return null. The best target is determined by the target sort mode in the PhotonVision UI.
        """
        raise NotImplementedError("best_target: Implement in subclass")

    @property
    def valid(self) -> bool:
        raise NotImplementedError("valid: Implement in subclass")

    @property
    def area(self) -> percent:
        """
        Target Area (0..100] percent of image
        """
        raise NotImplementedError("area: Implement in subclass")

    @property
    def x_offset(self) -> degrees:
        """
        Horizontal Offset from Crosshair to Target
        """
        raise NotImplementedError("x_offset: Implement in subclass")

    @property
    def y_offset(self) -> degrees:
        """
        Vertical Offset from Crosshair to Target
        """
        raise NotImplementedError("y_offset: Implement in subclass")

    def get_latest_results(self) -> Optional[Any]:
        #
        # TODO: When we implement the Limelight, see if we can get a generic
        #       structure along the lines of PhotonPipelineResults or better.
        #
        raise NotImplementedError("get_latest_results: Implement in subclass")

    def periodic(self):
        # Update inputs
        inputs: VisionIO.VisionIOInputs = self.inputs

        self.updateInputs(inputs)
        # TODO: Get this working.  TargetObservation failing  Logger.processInputs(f"Vision/Camera/{self.name}", inputs)

        # TODO: Once one subsystem supports multiple cameras, need to track 'all' poses
        #       instead of just one camera's worth.  See AdvantageKit vision.java example
        # # Initialize logging values
        # allTagPoses: List[Pose3d] = []
        # allRobotPoses: List[Pose3d] = []
        # allRobotPosesAccepted: List[Pose3d] = []
        # allRobotPosesRejected: List[Pose3d] = []

        # Update disconnected alert
        self._disconnected_alert.set(not inputs.connected)

        # Initialize logging values           TODO: Get these working
        tag_poses: List[Pose3d] = []
        robot_poses: List[Pose3d] = []
        robot_poses_accepted: List[Pose3d] = []
        robot_poses_rejected: List[Pose3d] = []

        # Add tag poses
        tag_ids = inputs.tag_ids or []

        for tag_id in tag_ids:
            tag_pose: Pose3d | None = self._field_layout.getTagPose(tag_id)
            if tag_pose:
                tag_poses.append(tag_pose)

        # Loop over pose observations
        observations: List[PoseObservation] = inputs.pose_observations or []
        for observation in observations:
            # Check whether to reject pose
            #   - Must have at least one tag
            #   - Cannot be high ambiguity
            #   - Must have realistic Z coordinate
            #   - Must be within the field boundaries
            x, y, z = observation.pose.X(), observation.pose.Y(), observation.pose.Z()

            reject_pose = observation.tag_count == 0 or \
                          (observation.tag_count == 1 and observation.ambiguity > constants.MAX_VISION_AMBIGUITY) or \
                          abs(z) > constants.MAX_VISION_Z_ERROR or \
                          x < 0.0 or \
                          y < 0.0 or \
                          x > self._field_layout.getFieldLength() or \
                          y > self._field_layout.getFieldWidth()

            # Add pose to log
            robot_poses.append(observation.pose)

            if reject_pose:
                robot_poses_rejected.append(observation.pose)
            else:
                robot_poses_accepted.append(observation.pose)

            # Skip if rejected
            if reject_pose:
                continue

            if self._estimate:
                # Calculate standard deviations
                # TODO: The CTRE drivetrain also supports vision standard deviations.  See how this and our
                #       new vision constants interact with those values
                #
                # TODO: Validate formulas below
                #
                std_dev_factor: float = math.pow(observation.avg_tag_distance, 2.0) / observation.tag_count
                linear_std_dev: float = constants.LINEAR_STD_DEV_BASELINE * std_dev_factor
                angular_std_dev: float = constants.ANGULAR_STD_DEV_BASELINE * std_dev_factor

                if observation.observation_type == PoseObservationType.MEGATAG_2:
                    linear_std_dev *= constants.LINEAR_STD_DEV_MEGATAG2_FACTOR
                    angular_std_dev *= constants.ANGULAR_STD_DEV_MEGATAG2_FACTOR

                # TODO: Support sending to drivetrain vision measurements here instead of elsewhere
                linear_std_dev *= self._std_dev_factor
                angular_std_dev *= self._std_dev_factor

                # Send vision observation
                self._drivetrain.add_vision_measurement(observation.pose, observation.timestamp,
                                                        (std_dev_factor, linear_std_dev, angular_std_dev))
        # Log camera metadata
        Logger.recordOutput(f"Vision/Camera/{self.name}/TagPoses", tag_poses)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPoses", robot_poses)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPosesAccepted", robot_poses_accepted)
        Logger.recordOutput(f"Vision/Camera/{self.name}/RobotPosesRejected", robot_poses_rejected)

        # TODO: Do following if we ever support multi-camera per subsystem,
        #   allTagPoses.extend(tagPoses);
        #   allRobotPoses.extend(robotPoses);
        #   allRobotPosesAccepted.extend(robotPosesAccepted);
        #   allRobotPosesRejected.extend(robotPosesRejected);
        #
        # Log summary data
        # Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[0]));
        # Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
        # Logger.recordOutput(
        #     "Vision/Summary/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
        # Logger.recordOutput(
        #     "Vision/Summary/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));

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
        pass
        # SmartDashboard.putString('Camera/heartbeat', "Alive" if self.heartbeating else "Dead")
        # SmartDashboard.putNumber('Camera/last-heartbeat', self.lastHeartbeatTime)

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
        pass
