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
#
#   2026 - Rebuilt      (All measuments are in meters)
#
import logging
import math

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters

# Setup Logging
logger = logging.getLogger(__name__)

FIELD_X_SIZE = 16.54
FIELD_Y_SIZE = 8.07
CENTER_LINE = FIELD_X_SIZE / 2
MID_FIELD = FIELD_Y_SIZE / 2

BLUE_START_LINE = inchesToMeters(182.11 - (47 / 2) - 2)
RED_START_LINE = FIELD_X_SIZE - inchesToMeters(182.11 - (47 / 2) - 2)

BLUE_BUMP_X_CENTER = inchesToMeters(182.11)
RED_BUMP_X_CENTER = FIELD_X_SIZE - inchesToMeters(182.11)

BLUE_TEST_POSE = {
    1: Pose2d(BLUE_START_LINE, 7.3, Rotation2d(math.pi)),
    2: Pose2d(BLUE_START_LINE, 6.16, Rotation2d(math.pi)),
    3: Pose2d(BLUE_START_LINE, 0.9, Rotation2d(math.pi))
}
RED_TEST_POSE = {
    1: Pose2d(RED_START_LINE, 0.9, 0),
    2: Pose2d(RED_START_LINE, 1.9, 0),
    3: Pose2d(RED_START_LINE, 7.3, 0)
}

# In simulation, the software will not enforce a maximum field
# size, so this needs to be accounted for so the robot stays on
# The field.
#
# The values below do not account the size of the robot

SIM_X_OFFSET_METERS = 0.140
SIM_Y_OFFSET_METERS = 0.95

# Add something with the AprilTags that are in this filed.  Call the drivetrain
# SetDesiredAprilTags() function.

json_file_path = '2026_field_layout.json'


# import robotpy_apriltag
#
# robotpy_apriltag.AprilTagFieldLayout.loadField()

class RebuiltField:
    """
    This class supports BLUE/RED alliances.

    When looking at the playing field, the origin is 0,0 (bottom left corner in landscape
    mode) with the Blue team on the left (lowest x-coordinate).  For the three teams in
    an alliance, we also assume

    NOTE: Any positional information will change each year based on the field. One other
          assumption is that

    All values are in meters
    """

    @staticmethod
    def in_blue_alliance_zone(x: float) -> bool:
        return x < inchesToMeters(182.11)

    @staticmethod
    def in_red_alliance_zone(x: float) -> bool:
        return x > FIELD_X_SIZE - inchesToMeters(182.11)

    @staticmethod
    def in_neutral_zone(x: float) -> bool:
        return inchesToMeters(182.11) < x < FIELD_X_SIZE - inchesToMeters(182.11)
