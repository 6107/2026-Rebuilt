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

from ntcore import DoublePublisher, NetworkTable
from wpimath.geometry import Pose3d, Rotation3d, Transform3d
from wpimath.units import degreesToRadians, meters

from lib_6107.pykit.LoggedMechanismObject2d import LoggedMechanismObject2d
from lib_6107.pykit.logtable import LogTable


class LoggedMechanismRoot2d:
    def __init__(self, name: str, x: meters, y: meters):
        """
        constructor for roots.

        @param name name
        @param x x coordinate of root (provide only when constructing a root node)
        @param y y coordinate of root (provide only when constructing a root node)
        """
        self._name = name
        self._x: meters = x
        self._y: meters = y
        self._objects: Dict[str, LoggedMechanismObject2d] = {}
        self._table: Optional[NetworkTable] = None
        self._x_publisher: DoublePublisher | None = None
        self._y_publisher: DoublePublisher | None = None

    def close(self) -> None:
        x_pub, self._x_publisher = self._x_publisher, None
        y_pub, self._y_publisher = self._y_publisher, None
        objects, self._objects = self._objects, {}

        if x_pub:
            x_pub.close()

        if y_pub:
            y_pub.close()

        for obj in objects.values():
            obj.close()

    def append(self, obj: LoggedMechanismObject2d) -> LoggedMechanismObject2d:
        name = obj.getName()
        if name in self._objects:
            raise ValueError(f"Mechanism objet names must be unique: {name}")

        self._objects[name] = obj

        if self._table is not None:
            obj.update(self._table.getSubTable(name))

        return obj

    def setPosition(self, x: meters, y: meters) -> None:
        """
        Set the root's position.

        @param x new x coordinate
        @param y new y coordinate
        """
        self._x, self._y = x, y
        self.flush()

    def update(self, table: NetworkTable) -> None:
        self._table = table

        if self._x_publisher is not None:
            self._x_publisher.close()

        self._x_publisher = table.getDoubleTopic("x").publish()

        if self._y_publisher is not None:
            self._y_publisher.close()

        self._y_publisher = table.getDoubleTopic("y").publish()
        self.flush()

        for obj in self._objects.values():
            obj.update(self._table.getSubTable(obj.getName()))

    def getName(self) -> str:
        """
        Get the name of the root.

        @return The name of the root.
        """
        return self._name

    def flush(self) -> None:
        if self._x_publisher is not None:
            self._x_publisher.set(self._x)

        if self._y_publisher is not None:
            self._y_publisher.set(self._y)

    def logOutput(self, table: LogTable) -> None:
        table.put("x", self._x)
        table.put("y", self._y)

        for obj in self._objects.values():
            obj.logOutput(table.getSubTable(obj.getName()))

    def generate3dMechanism(self) -> List[Pose3d]:
        """
        Converts the Mechanism2d into a series of Pose3d objects. Poses are generated with standard
        coordinate frame (+x forward, +y left, +z up) and each pivot point is assumed to be at the
        origin of the model.

        <p>The order of the poses returned is based on the order of insertion. The first root inserted
        into the Mechanism2d goes first, and processed in a depth-first manner.

        @return list of poses for starting from the root point
        """
        poses: List[Pose3d] = []

        # Coordinate shift changes from the xz plane to the xyz plane which is 'y' is 0
        initial_pose: Pose3d = Pose3d(self._x, 0, self._y, Rotation3d())

        for obj in self._objects.values():
            # convert mech2d angle to Rotation3d
            # remembering that +rotation in 2d is -pitch in 3d
            new_rotation = Rotation3d(0, degreesToRadians(-obj.getAngle()), 0)

            # Generate the pose for the next segment
            new_pose = Pose3d(initial_pose.translation(), new_rotation)
            poses.append(new_pose)

            # recurse down the length of that ligament
            next_pose = new_pose.transformBy(Transform3d(obj.getObject2dRange(), 0, 0, Rotation3d()))
            more_poses: List[Pose3d] = obj.generate3dMechanism(next_pose)
            poses.extend(more_poses)

        return poses
