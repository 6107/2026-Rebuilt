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

from dataclasses import dataclass, field

from pykit.autolog import autolog
from wpimath.geometry import Rotation2d


class GyroIO:
    @autolog
    @dataclass
    class GyroIOInputs:
        connected: bool = False
        yawPosition: Rotation2d = field(default_factory=lambda: Rotation2d())
        yawVelocityDegPerSec: float = 0.0

    def updateInputs(self, inputs: GyroIOInputs) -> None:
        pass
