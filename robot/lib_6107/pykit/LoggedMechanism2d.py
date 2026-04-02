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

from typing import Dict, List, Optional

from ntcore import DoubleArrayPublisher, NetworkTable, NTSendable, NTSendableBuilder, StringPublisher
from wpilib import Color, Color8Bit
from wpimath.geometry import Pose3d
from wpimath.units import meters

from lib_6107.pykit.LoggedMechanismRoot2d import LoggedMechanismRoot2d
from lib_6107.pykit.logtable import LogTable


class LoggedMechanism2d(NTSendable):
    def __init__(self, width: meters, height: meters,
                 backgroundColor: Optional[Color8Bit] = Color8Bit(Color.kDarkBlue)):
        """
        Create a new Mechanism2d with the given dimensions.

        <p>The dimensions represent the canvas that all the nodes are drawn on.

        @param width the width
        @param height the height
        @param backgroundColor the background color. Defaults to dark blue.
        """
        super().__init__()

        self._dimensions: List[meters] = [width, height]
        self._color: str = backgroundColor.hexString()
        self._roots: Dict[str, LoggedMechanismRoot2d] = {}

        self._table: Optional[NetworkTable] = None
        self._dimension_publisher: Optional[DoubleArrayPublisher] = None
        self._color_publisher: Optional[StringPublisher] = None

    def close(self) -> None:
        dim_pub, self._dimension_publisher = self._dimension_publisher, None
        color_pub, self._color_publisher = self._color_publisher, None
        roots, self._roots = self._roots, {}

        if dim_pub:
            dim_pub.close()

        if color_pub:
            color_pub.close()

        for root in roots.values():
            root.close()

    def getRoot(self, name: str, x: meters, y: meters) -> LoggedMechanismRoot2d | None:
        """
        Get or create a root in this Mechanism2d with the given name and position.
        <p>If a root with the given name already exists, the given x and y coordinates are not used.

        @param name the root name
        @param x the root x coordinate
        @param y the root y coordinate

        @return a new root joint object, or the existing one with the given name.
        """
        existing = self._roots.get(name)
        if existing:
            return existing

        root = LoggedMechanismRoot2d(name, x, y)
        self._roots[name] = root

        if self._table is not None:
            root.update(self._table.getSubTable(name))

        return root

    def setBackgroundColor(self, color: Color8Bit) -> None:
        """
         Set the Mechanism2d background color.

         @param color the new color
        """
        self._color = color.hexString()

    def initSendable(self, builder: NTSendableBuilder) -> None:
        builder.setSmartDashboardType("Mechanism2d")
        self._table = builder.getTable()

        if self._dimension_publisher is not None:
            self._dimension_publisher.close()

        self._dimension_publisher = self._table.getDoubleArrayTopic("dims").publish()
        self._dimension_publisher.set(self._dimensions)

        if self._color_publisher is not None:
            self._color_publisher.close()

        self._color_publisher = self._table.getStringTopic("backgroundColor").publish()
        self._color_publisher.set(self._color)

        for name, entry in self._roots.items():
            entry.update(self._table.getSubTable(name))

    def logOutput(self, table: LogTable) -> None:
        """
        Record the current value to the log. <b>This function should never be called by user code.</b>

        @param table The table to which data should be written.
        """
        table.put(".type", "Mechanism2d")
        table.put(".controllable", False)
        table.put("dims", self._dimensions)
        table.put("backgroundColor", self._color)

        for name, entry in self._roots.items():
            entry.logOutput(table.getSubTable(name))

    def generate3dMechanism(self) -> List[Pose3d]:
        """
        Converts a forward facing Mechanism2d into a series of Pose3d objects. Poses are generated with
        standard coordinate frame (+x forward, +y left, +z up) and each pivot point is assumed to be at
        the origin of the model.

        <p>The order of the poses returned is based on the order of insertion. The first root inserted
        into the Mechanism2d goes first, and processed in a depth-first manner.

        @return Pose3d[] representing each mechanism component
        """
        poses = []
        for root in self._roots.values():
            poses.extend(root.generate3dMechanism())

        return poses
