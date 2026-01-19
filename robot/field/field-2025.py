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
#       2025 - Reefscape
#
import logging
import math

from wpimath.geometry import Pose2d, Rotation2d
from wpimath.units import inchesToMeters

from lib_6107.util.game import field_flip_pose2d

# Setup Logging
logger = logging.getLogger(__name__)

FIELD_X_SIZE = 17.55
FIELD_Y_SIZE = 8.05
CENTER_LINE = FIELD_X_SIZE / 2
MID_FIELD = FIELD_Y_SIZE / 2

BLUE_START_LINE = inchesToMeters(144 - 14 + 93.5 + 88)
RED_START_LINE = FIELD_Y_SIZE - BLUE_START_LINE

BLUE_TEST_POSE = {
    1: Pose2d(BLUE_START_LINE, FIELD_Y_SIZE * 0.75, Rotation2d(math.pi)),
    2: Pose2d(BLUE_START_LINE, MID_FIELD, Rotation2d(math.pi)),
    3: Pose2d(BLUE_START_LINE, FIELD_Y_SIZE * 0.25, Rotation2d(math.pi))
}
RED_TEST_POSE = {
    1: field_flip_pose2d(BLUE_TEST_POSE[3]),
    2: field_flip_pose2d(BLUE_TEST_POSE[2]),
    3: field_flip_pose2d(BLUE_TEST_POSE[1])
}

# In simulation, the drawing includes the area around the playing field.
# You must manually account for it when moving the robot and keep the
# robots on the field, otherwise they will go off into the unknown.
#
# The values below do not account the size of the robot

SIM_X_OFFSET_METERS = 0.175
SIM_Y_OFFSET_METERS = 0.130

# BLUE_PODIUM_1 = Pose2d(Translation2d(0.48, CENTER_LINE - ), Rotation2d(math.pi))
# RED_PODIUM = field_flip_pose2d(BLUE_PODIUM)


class ReefScapeField:
    """
    This class supports BLUE/RED alliances.

    When looking at the playing field, the origin is 0,0 (bottom left corner in landscape
    mode) with the Blue team on the left (lowest x-coordinate).  For the three teams in
    an alliance, we also assume

    NOTE: Any positional information will change each year based on the field. One other
          assumption is that

    All values are in meters
    """
    pass
