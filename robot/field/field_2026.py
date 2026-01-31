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
#   2026 - Rebuilt      (All measurements are in metric units)

import logging
import math

from robotpy_apriltag import AprilTagField
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters, meters

from lib_6107.util.field import Field, FieldInfo

# Setup Logging
logger = logging.getLogger(__name__)

FIELD_X_SIZE = 16.54  # Field Length
FIELD_Y_SIZE = 8.07  # Field Width

CENTER_LINE = FIELD_X_SIZE / 2           # Divides the neutral zone
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

BLUE_HUB_X_OFFSET = inchesToMeters(182.11)
RED_HUB_X_OFFSET = FIELD_X_SIZE - BLUE_HUB_X_OFFSET

# In simulation, the software will not enforce a maximum field
# size, so this needs to be accounted for so the robot stays on
# The field.
#
# The values below do not account the size of the robot

SIM_X_OFFSET_METERS = 0.140
SIM_Y_OFFSET_METERS = 0.95


class RebuiltField(Field):
    """
    This class supports BLUE/RED alliances.

    When looking at the playing field, the origin is 0,0 (bottom left corner in landscape
    mode) with the Blue team on the left (lowest x-coordinate).  For the three teams in
    an alliance, we also assume

    NOTE: Any positional information will change each year based on the field. One other
          assumption is that

    All values are in meters
    """
    _field_info: FieldInfo = tuple([
        tuple(["Rebuilt (AndyMark)", AprilTagField.k2026RebuiltAndyMark, "2026-rebuilt-andymark.json"]),
        tuple(["Rebuilt (Welded)", AprilTagField.k2026RebuiltWelded, "2026-rebuilt-welded.json"]),
        tuple(["Custom", None, "2026-rebuilt-andymark.json"])
    ])

    def in_blue_alliance_zone(self, x: float) -> bool:
        return x < inchesToMeters(182.11)

    def in_red_alliance_zone(self, x: float) -> bool:
        return x > FIELD_X_SIZE - inchesToMeters(182.11)

    def in_neutral_zone(self, x: float) -> bool:
        return inchesToMeters(182.11) < x < FIELD_X_SIZE - inchesToMeters(182.11)

    @property
    def field_length(self) -> meters:
        """
        x maximum
        """
        return super().field_length or FIELD_X_SIZE

    @property
    def field_width(self) -> meters:
        """
        y maximum
        """
        return self._layout.getFieldWidth() or FIELD_Y_SIZE

    def hub_location(self, is_red_alliance: bool) -> Translation2d:
        if is_red_alliance:
            return Translation2d(x=self.field_length - BLUE_HUB_X_OFFSET, y=MID_FIELD)

        return Translation2d(x=BLUE_HUB_X_OFFSET, y=MID_FIELD)
