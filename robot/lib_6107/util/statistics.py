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

from collections import deque
from typing import Deque, Dict, Optional

from wpilib import RobotBase


class MovingAverage:
    def __init__(self, name: str, units: str = "", max_samples: int = 20, scale: int = 1, precision: int = 0):
        self.name = name
        self.units = units
        self.max_samples: int = max_samples
        self.scale: int = scale
        self.precision: int = precision

        self._samples: Deque[float] = deque(maxlen=max_samples)

    def clear(self) -> None:
        self._samples = deque(maxlen=self.max_samples)

    def add(self, value: float, count: Optional[bool] = True) -> None:
        # Add to end if we want to count it or if it is our first item
        if count or len(self._samples) == 0:
            self._samples.append(value)
        else:
            # Add it the last sample
            self._samples[-1] += value

    @property
    def average(self) -> float:
        length = len(self._samples)
        avg = sum(self._samples) / length if length > 0 else 0.0

        avg *= self.scale
        if self.precision:
            avg = round(avg, self.precision)

        return avg


class RobotStatistics:
    def __init__(self, robot: RobotBase):
        # Most stats are milliseconds (scale=1000) with resolution to a microsecond (precision=3)
        self._statistics: Dict[str, MovingAverage] = {
            "periodic-duration": MovingAverage("Periodic", units="S", max_samples=5),
            "teleop-duration"  : MovingAverage("Teleop", units="S", max_samples=5),
            "auto-duration"    : MovingAverage("Autonomous", units="S", max_samples=5),
        }
        self._robot: RobotBase = robot

    def get(self, name) -> MovingAverage | None:
        return self._statistics.get(name)

    def add(self, name, value: float, count: Optional[bool] = True) -> None:
        stats = self._statistics.get(name)
        if stats is None:
            return

        stats.add(value, count=count)

        # The robot's 'robotPeriodic' is called after the teleop/auto/.. version of the
        # periodic call. If this is a 'robotPeriodic' value, add it to the previously
        # value so the 'robotPeriodic' is charged to the right state.
        if not self._robot.isDisabled():
            match name:
                case "periodic":
                    if self._robot.isTeleop():
                        self.add("teleop", value, count=False)

                    elif self._robot.isAutonomous():
                        self.add("auto", value, count=False)

                case "periodic-duration":
                    if self._robot.isTeleop():
                        self.add("teleop-duration", value, count=False)

                    elif self._robot.isAutonomous():
                        self.add("auto-duration", value, count=False)

    def clear(self, name) -> None:
        if name == "all":
            for stat in self._statistics.values():
                stat.clear()

        elif name in self._statistics:
            self._statistics[name].clear()
