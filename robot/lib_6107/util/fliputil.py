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
#  Credit for this is Westwood Robotics.  Look them up...
#
from math import pi

from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

from robot_2026.field.field_2026 import Field


class FlipUtil:
    """
    Utility for flipping wpilib geometry classes based on current alliance
    """

    def __init__(self, field: Field):
        self._field = field

    @staticmethod
    def should_flip() -> bool:
        """
        Determine whether to flip based on current alliance

        :return: True if should flip, False otherwise
        """
        return DriverStation.getAlliance() == DriverStation.Alliance.kRed

    @staticmethod
    def field_pose(pose: Pose2d) -> Pose2d:
        """
        Flip a Pose2d based on current alliance

        :param pose: The Pose2d to potentially flip
        :return: The flipped or original Pose2d
        """
        return Pose2d(
            FlipUtil.field_translation(pose.translation()),
            FlipUtil.field_rotation(pose.rotation()),
        )

    def field_translation(self, translation: Translation2d) -> Translation2d:
        """
        Flip a Translation2d based on current alliance

        :param translation: The Translation2d to potentially flip
        :return: The flipped or original Translation2d
        """
        if FlipUtil.should_flip():
            return Translation2d(self._field.field_length, self._field.field_width) - translation
        return translation

    @staticmethod
    def field_rotation(rotation: Rotation2d) -> Rotation2d:
        """
        Flip a Rotation2d based on current alliance

        :param rotation: The Rotation2d to potentially flip
        :return: The flipped or original Rotation2d
        """
        return rotation - Rotation2d(pi) if FlipUtil.should_flip() else rotation
