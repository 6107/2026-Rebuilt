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

from typing import Tuple

from pykit.logger import Logger
from wpilib import RobotBase
from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import radians_per_second, seconds, meters
from wpimath.kinematics import ChassisSpeeds, SwerveDrive4Odometry, SwerveModulePosition

from constants import DriveKinematics
from util.logtracer import LogTracer


ModulePositionType = Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]


class RobotState:
    auto_distance_tolerance: meters = 0.1
    auto_rotation_tolerance: meters = 0.1

    robot_pose: Pose2d = Pose2d()
    heading_offset: Rotation2d = Rotation2d()
    robot_heading: Rotation2d = Rotation2d()

    module_positions: ModulePositionType = (SwerveModulePosition(), SwerveModulePosition(),
                                            SwerveModulePosition(), SwerveModulePosition())

    odometry: SwerveDrive4Odometry = SwerveDrive4Odometry(DriveKinematics, Rotation2d(),
                                                          module_positions, Pose2d())
    robot_field_velocity: ChassisSpeeds = ChassisSpeeds()

    @classmethod
    def periodic(cls, pose: Pose2d,
                 heading: Rotation2d,
                 heading_timestamp: seconds,
                 yaw_rate: radians_per_second,
                 field_relative_robot_velocity: ChassisSpeeds,
                 module_positions: Tuple[SwerveModulePosition, SwerveModulePosition, SwerveModulePosition, SwerveModulePosition]) -> None:

        LogTracer.resetOuter("RobotState")

        cls.robot_heading = heading
        cls.module_positions = module_positions
        cls.robot_pose = pose

        # TODO: Start here tomorrow
        estimated_field_pose = cls.odometry.update(heading, module_positions)

        cls.robot_field_velocity = field_relative_robot_velocity

        LogTracer.record("OdometryUpdate")

        Logger.recordOutput("Robot/Pose/EstimatorPose", estimated_field_pose)
        Logger.recordOutput("Robot/Pose/OdometryPose", pose)

        Logger.recordOutput("Robot/Heading", cls.robot_heading)
        Logger.recordOutput("Robot/HeadingVelocity", yaw_rate)
        Logger.recordOutput("Robot/Velocity", field_relative_robot_velocity)
        Logger.recordOutput("Robot/HeadingOffset", cls.heading_offset)

        position_delta = pose - estimated_field_pose

        Logger.recordOutput("Auto/PositionOffset", position_delta)
        Logger.recordOutput("Auto/PositionCorrect",
                            position_delta.translation().norm() < cls.auto_distance_tolerance)
        Logger.recordOutput("Auto/RotationCorrect",
                            abs(position_delta.rotation().radians()) < cls.auto_rotation_tolerance)

        # TODO: Should we log starting location in AutonomousInit?  Would that be metadata

        LogTracer.recordTotal()

    @classmethod
    def get_pose(cls) -> Pose2d:
        return cls.estimator.estimatedPose      # TODO: Is this called?  May need it commands

    @classmethod
    def get_rotation(cls) -> Rotation2d:
        return cls.robot_pose.rotation()

    @classmethod
    def reset_pose(cls, pose: Pose2d = Pose2d()) -> None:  # TODO: Is this called?  May need it commands
        cls.heading_offset = cls.robot_heading - pose.rotation()
        cls.odometry.resetPosition(cls.robot_heading, cls.module_positions, pose)
