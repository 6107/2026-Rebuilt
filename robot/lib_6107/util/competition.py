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
from collections import deque
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Deque, Iterable, Tuple

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class EventStartEndTime:
    start_time: datetime
    end_time: datetime


class Event:
    """
    A single event.

    The dates can be a single timespan or multiple.  The date and time will be evaluated
    in the date/time that the RoboRio is running in. For that reason, all values must have
    a timezone defined.

    Also, start times should be before you turn the robot on for the first time...
    """

    def __init__(self, title: str, dates: EventStartEndTime | Iterable[EventStartEndTime]) -> None:
        self._title = title

        if isinstance(dates, EventStartEndTime):
            dates = [dates]

        self._dates: Tuple[EventStartEndTime, ...] = tuple(dates)
        self._validate_date_times()

    @property
    def title(self) -> str:
        return str(self._title)

    @property
    def dates(self) -> Tuple[EventStartEndTime, ...]:
        return self._dates

    @property
    def active(self) -> bool:
        """
        Are we in the middle of an event?
        """
        now = datetime.now(timezone.utc)
        return any(dt.start_time <= now <= dt.end_time for dt in self._dates)

    def _validate_date_times(self) -> None:
        """
        Check what was provided. Make sure start < end for each time and has a timezone defined.
        """

        def has_timezone(dt_object: datetime):
            """
            Checks if a datetime object has a timezone defined
            """
            return dt_object.tzinfo is not None

        for index, date in enumerate(self._dates):
            assert has_timezone(date.start_time), f"Start time {index} of {self.title} Does not " \
                                                  f"have a timezone defined: {date.start_time}"

            assert has_timezone(date.end_time), f"End time {index} of {self.title} Does not " \
                                                f"have a timezone defined: {date.start_time}"

            assert date.start_time < date.end_time, f"Start > End for item {index} of " \
                                                    f"{self.title}. {date.start_time} >= {date.end_time}"


###############################################################################################
# Track the competitions here
__events: Deque[Event] = deque()


def add_event(competition: Event):
    """
    Add an event/competition to the list of events.
    """
    global __events
    __events.append(competition)


def event_active() -> bool:
    """
    Is a registered event or competition active?
    """
    return True
    # global __events
    # return any(event.active for event in __events)


competition_active = event_active  # Events and completions are the same
add_competition = add_event  # Events and completions are the same
