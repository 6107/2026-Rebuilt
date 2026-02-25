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

from dataclasses import dataclass

from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, seconds


class GyroIO:
    @autolog
    @dataclass
    class GyroIOInputs:
        """
        Loggable inputs for a gyro sensor.

        Currently only the yaw (angle) and yaw rate
        """
        connected: bool = False
        yaw: radians = 0.0
        yaw_rate: radians_per_second = 0.0

        roll: radians = 0.0
        pitch: radians = 0.0

        # Following optional and only valid if non-zero
        yaw_timestamp: seconds = 0.0
        yaw_rate_timestamp: seconds = 0.0
        roll_timestamp: seconds = 0.0
        pitch_timestamp: seconds = 0.0

    def updateInputs(self, inputs: GyroIOInputs) -> None:
        pass

    def set_yaw(self, yaw_rad: radians) -> None:
        """
        Set the gyro yaw.

        Args:
            yaw_rad (float): The yaw in radians.
        """
