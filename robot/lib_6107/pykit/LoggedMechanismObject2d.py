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

from typing import Dict, List

from ntcore import NetworkTable
from wpimath.geometry import Pose3d, Rotation3d, Transform3d
from wpimath.units import degrees, degreesToRadians

from lib_6107.pykit.logtable import LogTable


class LoggedMechanismObject2d:
    def __init__(self, name: str):
        """
        Create a new Mechanism node object.

        @param name the node's name, must be unique.
        """
        self._name = name
        self._table = None
        self._objects: Dict[str, LoggedMechanismObject2d] = {}

    def close(self) -> None:
        objects, self._objects = self._objects, {}
        for obj in self._objects.values():
            obj.close()

    def append(self, obj: LoggedMechanismObject2d) -> LoggedMechanismObject2d:
        """
        Append a Mechanism object that is based on this one.

        @param <T> The object type.
        @param object the object to add.
        @return the object given as a parameter, useful for variable assignments and call chaining.
        @throws UnsupportedOperationException if the object's name is already used - object names must be unique.
        """
        name = obj._name
        if name in self._objects:
            raise ValueError(f"Mechanism objet names must be unique: {name}")

        self._objects[name] = obj

        if self._table is not None:
            obj.update(self._table.getSubTable(name))

        return obj

    def update(self, table: NetworkTable) -> None:
        self._table = table
        self.updateEntries(table)

        for obj in self._objects.values():
            obj.update(self._table.getSubTable(obj.getName()))

    def updateEntries(self, table: NetworkTable) -> None:
        raise NotImplementedError("updateEntries: Implement in a subclass")

    def getName(self) -> str:
        return self._name

    def logOutput(self, table: LogTable) -> None:
        for obj in self._objects.values():
            obj.logOutput(table.getSubTable(obj.getName()))

    def generate3dMechanism(self, seed: Pose3d) -> List[Pose3d]:
        """
        Propagates the mechanism2d down the tree structure.

        @param seed position to start the calculations at
        @return array list of all poses generated from this point in a depth-first pattern
        """

        poses: List[Pose3d] = []
        initial_pose = seed

        for obj in self._objects.values():
            # convert mech2d angle to Rotation3d
            # remembering that +rotation in 2d is -pitch in 3d
            new_rotation = Rotation3d(0, degreesToRadians(-obj.getAngle()), 0)

            # Generate the pose for the new joint
            new_pose = Pose3d(initial_pose.translation(), new_rotation)
            poses.append(new_pose)

            # recurse down the length of that ligament
            transform: Transform3d = Transform3d(obj.getObject2dRange(), 0, 0, Rotation3d())
            next_pose: Pose3d = new_pose.transformBy(transform)

            more_poses = obj.generate3dMechanism(next_pose)
            poses.extend(more_poses)

        return poses

    def getObject2dRange(self) -> float:
        """
        Abstract helper function. A proxy for getLength() with Ligament2d, but would be something else
        like getRadius() for circular parts if they were to be implemented.

        @return distance in meters
        """
        raise NotImplementedError("getObject2dRange: Implement in a subclass")

    def getAngle(self) -> degrees:
        """
        Abstract helper function. Should be common to all 2d parts, and assumes a normal xy or xz
        positive direction of left or up, respectively.

        @return angle in degrees
        """
        raise NotImplementedError("getAngle: Implement in a subclass")
