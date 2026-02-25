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
from typing import Optional

from commands2.button import Trigger
from robotpy_apriltag import AprilTagField
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.units import inchesToMeters, meters, seconds

from lib_6107.util.field import Field, FieldInfo

# Setup Logging
logger = logging.getLogger(__name__)

# First some game constants

AUTONOMOUS_DURATION: seconds = 20  # Both hubs active

TELEOP_DURATION: seconds = 140
TRANSITION_DURATION: seconds = 10  # Both hubs active
SHIFT_DURATION: seconds = 25  # Total of 4 shifts
END_GAME_DURATION: seconds = 30  # Both hubs active

# Now this year's field

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
    an alliance, we also assume Blue 1 is top left, Red 1 is bottom right.

    NOTE: Any positional information will change each year based on the field.

    All values are in meters.
    """
    _field_info: FieldInfo = tuple([
        # Default for a chooser will be the first entry below.
        tuple(["Rebuilt (Welded)", AprilTagField.k2026RebuiltWelded, "2026-rebuilt-welded.json"]),
        tuple(["Rebuilt (AndyMark)", AprilTagField.k2026RebuiltAndyMark, "2026-rebuilt-andymark.json"]),
    ])

    def __init__(self):
        super().__init__()

        self._won_autonomous: Optional[bool] = None


    def in_blue_alliance_zone(self, x: float) -> bool:
        return x < inchesToMeters(182.11)

    def in_red_alliance_zone(self, x: float) -> bool:
        return x > self.field_length - inchesToMeters(182.11)

    def in_neutral_zone(self, x: float) -> bool:
        return inchesToMeters(182.11) < x < self.field_length - inchesToMeters(182.11)

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

    @property
    def autonomous_winner(self) -> DriverStation.Alliance | None:
        """
        Returns the alliance that won autonomous, or None if unknown
        This is determined by the game specific message sent by the field
        https://docs.wpilib.org/en/stable/docs/yearly-overview/2026-game-data.html
        """
        match DriverStation.getGameSpecificMessage():
            case "R":
                return DriverStation.Alliance.kRed

            case "B":
                return DriverStation.Alliance.kBlue

            case None | "":
                return None

            case _:
                logger.error(f"Unknown alliance message: {DriverStation.getGameSpecificMessage()}")
                return None

    @property
    def won_autonomous(self) -> Optional[bool]:
        """
        Returns true if our alliance won autonomous, false otherwise. None if unknown.
        """
        if self._won_autonomous is None:
            alliance = DriverStation.getAlliance()
            winner = self.autonomous_winner

            if alliance is None or winner is None:
                return None

            self._won_autonomous = winner == alliance

        return self._won_autonomous

    # TODO: Maybe have the controllers 'rumble' when we transition
    #       phases of the game.  Maybe small rumble 5 seconds to go

    @property
    def hub_active(self) -> bool:
        """
        Returns true if the active hub is the one we are scoring on.

        The active hub is determined by the match time and whether we won autonomous
        0-20 seconds: Autonomous, both hubs active

        21-110 seconds: Shift periods, only one hub active
            Shift 1 (86-110s): Hub determined by autonomous winner
            Shift 2 (61-85s):  Hub opposite of autonomous winner
            Shift 3 (36-60s):  Hub determined by autonomous winner
            Shift 4 (21-35s):  Hub opposite of autonomous winner

        111-140 seconds: Endgame, both hubs active
        """
        if DriverStation.isAutonomous():
            return True

        # We are in the Teleop period
        #
        # Get the time remaining in current match period (auto or teleop) in seconds. This
        # value counts down towards zero.
        time = DriverStation.getMatchTime()

        # Is the endgame or the Transition Shift?  Both hubs active at that time
        if time <= END_GAME_DURATION or time >= TELEOP_DURATION - TRANSITION_DURATION:
            return True

        # Do not know. Should not really get here in a real game as the Transision Shift
        # period is 10 seconds and the FMS should have figured things out by now.
        if self._won_autonomous is None:
            return False

        # Trim off the end-game-time so comparisons are easy to do
        time -= END_GAME_DURATION

        # Shift 4: Hub that won autonomous has an active hub
        if time <= SHIFT_DURATION:
            return self._won_autonomous

        time -= SHIFT_DURATION

        # Shift 3: Hub that DID NOT win autonomous has an active hub
        if time <= SHIFT_DURATION:
            return not self._won_autonomous

        time -= SHIFT_DURATION

        # Shift 2: Hub that won autonomous has an active hub
        if time <= SHIFT_DURATION:
            return self._won_autonomous

        # Must be shift 1: Hub that DID NOT win autonomous has an active hub
        return not self._won_autonomous

    @property
    def shift_trigger(self) -> Trigger:
        """
        Returns a trigger that is active when the hub we are scoring on is active
        This is used for command based programming to enable/disable commands based on hub activity
        See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        """
        return Trigger(self.hub_active is True)
