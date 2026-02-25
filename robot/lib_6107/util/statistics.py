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

from typing import Dict, Optional


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

    def clear(self) -> None:
        self.max = None
        self.min = None
        self.total = None
        self.count = 0

    def add(self, value: float) -> None:
        if self.count:
            self.total += value
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

            avg = self.total / self.count
            print(f"{blanks}{self.indent}  Min: {value(self.min)} {self.units}")
            print(f"{blanks}{self.indent}  Max: {value(self.max)} {self.units}")
            print(f"{blanks}{self.indent}  Avg: {value(avg)} {self.units} over {self.count} samples")
            print(f"{blanks}{self.indent}Total: {round(self.total, 6)} Seconds")
        print()

_unknown_status = MaxMinCounter("unknown")

class RobotStatistics:
    def __init__(self):

        # Most stats are milliseconds (scale=1000) with resolution to a microsecond (precision=3)
        self._statistics: Dict[str, MaxMinCounter] = {
            "periodic": MaxMinCounter("Periodic Duration", "mS", 1000, 3),
            "disabled": MaxMinCounter("Disabled Duration", "mS", 1000, 3),
            "teleop":   MaxMinCounter("Teleop Duration", "mS", 1000, 3),
            "auto":     MaxMinCounter("Autonomous Duration", "mS", 1000, 3),
            "sim":      MaxMinCounter("Update Sim Duration", "mS", 1000, 3),
        }

    def add(self, name, value: float) -> None:
        stats = self._statistics.get(name, _unknown_status)
        stats.add(value)

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
