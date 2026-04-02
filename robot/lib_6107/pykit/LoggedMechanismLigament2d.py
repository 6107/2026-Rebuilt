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

from typing import Optional

from ntcore import DoubleEntry, NetworkTable, StringEntry, StringPublisher
from wpilib import Color8Bit
from wpimath.geometry import Rotation2d
from wpimath.units import degrees, meters

from lib_6107.pykit.LoggedMechanismObject2d import LoggedMechanismObject2d
from lib_6107.pykit.logtable import LogTable


class LoggedMechanismLigament2d(LoggedMechanismObject2d):

    def __init__(self, name: str, length: meters, angle: degrees, linewidth: Optional[float] = 10,
                 color: Optional[Color8Bit] = None):

        super().__init__(name)
        self._type_pub: StringPublisher | None = None
        self._angle_entry: DoubleEntry | None = None
        self._color_entry: StringEntry | None = None
        self._length_entry: DoubleEntry | None = None
        self._weight_entry: DoubleEntry | None = None

        self._angle: degrees = angle

        color = color or Color8Bit(235, 137, 52)
        self._color: str = color.hexString()

        self._length: meters = length

        self._weight: float = linewidth

    def close(self) -> None:
        super().close()

        type_pub, self._type_pub = self._type_pub, None
        angle_entry, self._angle_entry = self._angle_entry, None
        color_entry, self._color_entry = self._color_entry, None
        length_entry, self._length_entry = self._length_entry, None
        weight_entry, self._weight_entry = self._weight_entry, None

        if type_pub is not None:
            type_pub.close()

        if angle_entry is not None:
            angle_entry.close()

        if color_entry is not None:
            color_entry.close()

        if length_entry is not None:
            length_entry.close()

        if weight_entry is not None:
            weight_entry.close()

    def setAngle(self, angle: degrees | Rotation2d) -> None:
        if isinstance(angle, Rotation2d):
            angle = angle.degrees()

        self._angle = angle

        if self._angle_entry is not None:
            self._angle_entry.set(angle)

    def getAngle(self) -> degrees:
        if self._angle_entry is not None:
            self._angle = self._angle_entry.get()

        return self._angle

    def setLength(self, length: meters) -> None:
        self._length = length

        if self._length_entry is not None:
            self._length_entry.set(length)

    def getLength(self) -> meters:
        if self._length_entry is not None:
            self._length = self._length_entry.get()

        return self._length

    def setColor(self, color: Color8Bit) -> None:
        self._color = color.hexString()

        if self._color_entry is not None:
            self._color_entry.set(self._color)

    def getColor(self) -> Color8Bit:
        if self._color_entry is not None:
            self._color = self._color_entry.get()

        return Color8Bit().fromHexString(self._color)

    def setLineWeight(self, weight: float) -> None:
        self._weight = weight

        if self._weight_entry is not None:
            self._weight_entry.set(self._weight)

    def getLineWeight(self) -> float:
        if self._weight_entry is not None:
            self._weight = self._weight_entry.get()

        return self._weight

    def updateEntries(self, table: NetworkTable) -> None:
        if self._type_pub is not None:
            self._type_pub.close()

        self._type_pub = table.getStringTopic(".type").publish()

        if self._angle_entry is not None:
            self._angle_entry.close()

        self._angle_entry = table.getDoubleTopic("angle").getEntry(0.0)

        if self._length_entry is not None:
            self._length_entry.close()

        self._length_entry = table.getDoubleTopic("length").getEntry(0.0)

        if self._color_entry is not None:
            self._color_entry.close()

        self._color_entry = table.getStringTopic("color").getEntry("")

        if self._weight_entry is not None:
            self._weight_entry.close()

        self._weight_entry = table.getDoubleTopic("weight").getEntry(0.0)

    def logOutput(self, table: LogTable) -> None:
        table.put(".type", "line")
        table.put("angle", self._angle)
        table.put("length", self._length)
        table.put("color", self._color)
        table.put("weight", self._weight)

        super().logOutput(table)

    def getObject2dRange(self) -> meters:
        return self.getLength()
