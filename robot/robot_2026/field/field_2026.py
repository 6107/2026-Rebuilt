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
from enum import auto, IntEnum
from typing import Optional

from commands2.button import Trigger
from robotpy_apriltag import AprilTagField
from wpilib import DriverStation
from wpimath.geometry import Pose2d, Pose3d, Rotation2d, Rotation3d, Transform3d, Translation2d, Translation3d
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

SHIFT_CHANGE_DELTA_BEFORE: int = 3
SHIFT_CHANGE_DELTA_AFTER: int = 5

# Now this year's field

FIELD_X_SIZE: meters = 16.54  # Field Length
FIELD_Y_SIZE: meters = 8.07  # Field Width

CENTER_LINE: meters = FIELD_X_SIZE / 2  # Divides the neutral zone
MID_FIELD: meters = FIELD_Y_SIZE / 2  # Divides left/right (lengthwise) zone

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

BLUE_HUB_X_OFFSET: meters = inchesToMeters(182.11)
RED_HUB_X_OFFSET: meters = FIELD_X_SIZE - BLUE_HUB_X_OFFSET

# In simulation, the software will not enforce a maximum field
# size, so this needs to be accounted for so the robot stays on
# The field.
#
# The values below do not account the size of the robot

SIM_X_OFFSET_METERS: meters = 0.140
SIM_Y_OFFSET_METERS: meters = 0.95

# TODO: Maybe best to have item below in a different constants file closer to subsystem that supplies it
SHOOTER_LOCATION = Transform3d(  # TODO: Need to confim
    Translation3d(-0.102, 0.178, 0.368),
    Rotation3d(),  # In cad this is the center of the top most plate on the flywheel output (if we had CAD)
)

BLUE_HUB_LOCATION = Translation2d(BLUE_HUB_X_OFFSET, CENTER_LINE)
RED_HUB_LOCATION = Translation2d(RED_HUB_X_OFFSET, CENTER_LINE)

class FieldLocation(IntEnum):
    ALLIANCE_LEFT = auto()
    ALLIANCE_RIGHT = auto()
    NEUTRAL_LEFT = auto()
    NEUTRAL_RIGHT = auto()
    OPPONENT_LEFT = auto()
    OPPONENT_RIGHT = auto()


def pose3d_from2d(pose: Pose2d) -> Pose3d:
    return Pose3d(pose.X(), pose.Y(), 0, Rotation3d(0, 0, pose.rotation().radians()))


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
        self._hub_location: Translation2d | None = None
        self._alliance: DriverStation.Alliance | None = DriverStation.getAlliance()

        from lib_6107.util.fliputil import FlipUtil
        self._flip_util: FlipUtil = FlipUtil(self)

        # TODO: Need to implement...
        self._field_estimator = None
        self._hub_estimator = None

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

    @property
    def field_pose(self) -> Pose2d:
        return self._field_estimator.estimatedPose


    @property
    def hub_pose(self) -> Pose2d:
        return self._hub_estimator.estimatedPose if self._hub_estimator is not None else Pose2d()

    @property
    def hub_location(self) -> Translation2d | None:  # TODO: Validate callers handle 'None' case
        # if is_red_alliance:
        #     return Translation2d(x=self.field_length - BLUE_HUB_X_OFFSET, y=MID_FIELD)
        #
        # return Translation2d(x=BLUE_HUB_X_OFFSET, y=MID_FIELD)
        """
        Returns the global field-space location of the hub as a Translation2d from the origin, based on the alliance color
        """
        if self._hub_location is None:
            match self._alliance or DriverStation.getAlliance():
                case DriverStation.Alliance.kRed:
                    self._hub_location = RED_HUB_LOCATION

                case DriverStation.Alliance.kBlue:
                    self._hub_location = BLUE_HUB_LOCATION

        return self._hub_location

    @property
    def distance_to_hub(self) -> meters | None:  # TODO: Validate callers handle 'None' case
        """
        Returns the distance from the robot to the hub in meters, based on the hub estimator
        """
        location = self.hub_location
        if location is not None:
            location.distance((pose3d_from2d(self.hub_pose) + SHOOTER_LOCATION)
                              .toPose2d().translation())
        return None

    #############################################################
    #  Status and Triggers important to the match

    @property
    def autonomous_winner(self) -> DriverStation.Alliance | None:  # TODO: Validate callers handle 'None' case
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
    def won_autonomous(self) -> bool | None:  # TODO: Validate callers handle 'None' case
        """
        Returns true if our alliance won autonomous, false otherwise. None if unknown.
        """
        if self._won_autonomous is None:
            alliance = self._alliance or DriverStation.getAlliance()
            winner = self.autonomous_winner

            if alliance is None or winner is None:
                return None

            self._won_autonomous = winner == alliance

        return self._won_autonomous

    @property
    def hub_about_to_change(self) -> bool:
        """
        Returns true during 3 seconds before a hub change, which happens at
        2:10, 1:45, 1:20, 55, and 30 seconds remaining
        """
        time = DriverStation.getMatchTime()
        if time <= 0:
            return False  # safety net for negative time, assume it's not about to change (testing)

        return any(END_GAME_DURATION + SHIFT_DURATION * i + SHIFT_CHANGE_DELTA_BEFORE >=
                   time >=
                   END_GAME_DURATION + SHIFT_DURATION * i for i in range(SHIFT_CHANGE_DELTA_AFTER))

    # TODO: Maybe have the controllers 'rumble' when we transition
    #       phases of the game.  Maybe small rumble 5 seconds to go
    @property
    def should_go_to_hub(self) -> bool:
        return self.hub_about_to_change is True and self.hub_active is False

    @property
    def should_go_to_feed(self) -> bool:
        time = DriverStation.getMatchTime()
        if time <= 0:
            return False  # safety net for negative time, assume it's not about to change (testing)

        return (
                any(
                    END_GAME_DURATION + SHIFT_DURATION * i - SHIFT_CHANGE_DELTA_BEFORE
                    >= time
                    >= END_GAME_DURATION + SHIFT_DURATION * i - SHIFT_CHANGE_DELTA_AFTER
                    for i in range(0, SHIFT_CHANGE_DELTA_AFTER)
                )
                and not self.hub_active
        )

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
    def hub_about_to_change_trigger(self) -> Trigger:
        return Trigger(lambda: self.hub_about_to_change)

    @property
    def should_go_to_hub_trigger(self) -> Trigger:
        return Trigger(lambda: self.should_go_to_hub)

    @property
    def should_go_to_feed_trigger(self) -> Trigger:
        return Trigger(lambda: self.should_go_to_feed)

    @property
    def shift_trigger(self) -> Trigger:
        """
        Returns a trigger that is active when the hub we are scoring on is active
        This is used for command based programming to enable/disable commands based on hub activity
        See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
        """
        return Trigger(self.hub_active is True)

    #############################################################
    #  What area (sub-area) of the field are we in
    def location(self, pose: Pose2d) -> FieldLocation | None:

        if self.in_left_zone_area(pose.y):
            if self.in_my_alliance_zone(pose.x):
                return FieldLocation.ALLIANCE_LEFT

            if self.in_neutral_zone(pose.x):
                return FieldLocation.NEUTRAL_LEFT

            if self.in_my_opponents_zone(pose.x):
                return FieldLocation.OPPONENT_LEFT

        if self.in_left_zone_area(pose.y) is False:
            if self.in_my_alliance_zone(pose.x):
                return FieldLocation.ALLIANCE_RIGHT

            if self.in_neutral_zone(pose.x):
                return FieldLocation.NEUTRAL_RIGHT

            if self.in_my_opponents_zone(pose.x):
                return FieldLocation.OPPONENT_RIGHT

        return None

    def in_blue_alliance_zone(self, x: meters) -> bool:
        return x < inchesToMeters(182.11)

    def in_red_alliance_zone(self, x: meters) -> bool:
        return x > self.field_length - inchesToMeters(182.11)

    def in_neutral_zone(self, x: meters) -> bool:
        return inchesToMeters(182.11) < x < self.field_length - inchesToMeters(182.11)

    def in_my_alliance_zone(self, x: meters) -> bool | None:  # TODO: Validate callers handle 'None' case
        match self._alliance or DriverStation.getAlliance():
            case DriverStation.Alliance.kRed:
                return self.in_red_alliance_zone(x)

            case DriverStation.Alliance.kBlue:
                return self.in_blue_alliance_zone(x)
        return None

    def in_my_opponents_zone(self, x: meters) -> bool | None:  # TODO: Validate callers handle 'None' case
        match self._alliance or DriverStation.getAlliance():
            case DriverStation.Alliance.kRed:
                return self.in_blue_alliance_zone(x)

            case DriverStation.Alliance.kBlue:
                return self.in_red_alliance_zone(x)
        return None

    def in_left_zone_area(self, y: meters) -> bool | None:  # TODO: Validate callers handle 'None' case
        match self._alliance or DriverStation.getAlliance():
            case DriverStation.Alliance.kBlue:
                return MID_FIELD < y <= FIELD_Y_SIZE

            case DriverStation.Alliance.kRed:
                return 0 <= y <= MID_FIELD

        return None

    def in_right_zone_area(self, y: meters) -> bool | None:  # TODO: Validate callers handle 'None' case
        left_zone = self.in_left_zone_area(y)

        return None if left_zone is None else not left_zone
