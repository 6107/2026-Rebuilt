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


class MaxMinCounter:
    indent = "  "
    def __init__(self, name: str, units: str = "", scale: int = 1, precision: int = 0):
        self.name = name
        self.units = units
        self.scale: int = scale
        self.precision: int = precision
        self.count: int = 0

        self.max: Optional[int | float] = None
        self.min: Optional[int | float] = None
        self.total: Optional[int | float] = None

    @property
    def average(self) -> float:
        return self.total / self.count if self.count > 0 else 0

    def clear(self) -> None:
        self.max = None
        self.min = None
        self.total = None
        self.count = 0

    def add(self, value: float, count: Optional[bool] = True) -> None:
        if self.count:
            self.total += value
            if count:
                self.count += 1
            self.max = max(value, self.max)
            self.min = min(value, self.min)
        else:
            self.count = 1
            self.total = self.max = self.min = value

    def print(self, depth: int = 0) -> None:
        blanks = self.indent * depth
        print(f"{blanks}{self.name}:")

        if self.count == 0:
            print(f"{blanks}{self.indent}No statistics available")
        else:
            def value(val: int | float) -> int | float:
                val *= self.scale
                if self.precision and isinstance(val, float):
                    val = round(val, self.precision)
                return val

            print(f"{blanks}{self.indent}  Min: {value(self.min)} {self.units}")
            print(f"{blanks}{self.indent}  Max: {value(self.max)} {self.units}")
            print(f"{blanks}{self.indent}  Avg: {value(self.average)} {self.units} over {self.count} samples")
            print(f"{blanks}{self.indent}Total: {round(self.total, 6)} Seconds")
        print()

_unknown_status = MaxMinCounter("unknown")


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
        self._statistics: Dict[str, MaxMinCounter | MovingAverage] = {
            "periodic": MaxMinCounter("Periodic Duration", "mS", 1000, 3),
            "disabled": MaxMinCounter("Disabled Duration", "mS", 1000, 3),
            "teleop":   MaxMinCounter("Teleop Duration", "mS", 1000, 3),
            "auto":     MaxMinCounter("Autonomous Duration", "mS", 1000, 3),
            "sim":      MaxMinCounter("Update Sim Duration", "mS", 1000, 3),

            "periodic-duration": MovingAverage("Periodic", units="S", max_samples=50),
            "teleop-duration"  : MovingAverage("Teleop", units="S", max_samples=50),
            "auto-duration"    : MovingAverage("Autonomous", units="S", max_samples=50),
        }
        self._robot: RobotBase = robot

    def get(self, name) -> MaxMinCounter | MovingAverage | None:
        return self._statistics.get(name)

    def add(self, name, value: float, count: Optional[bool] = True) -> None:
        stats = self._statistics.get(name, _unknown_status)
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

    def print(self, name, depth: int = 0) -> None:
        if name == "all":
            for stat in self._statistics.values():
                stat.print(depth)

        elif name in self._statistics:
            self._statistics[name].print(depth)
