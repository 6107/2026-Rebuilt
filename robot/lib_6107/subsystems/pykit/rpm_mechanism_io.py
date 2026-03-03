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
from wpimath.units import amperes, radians, radians_per_second, volts


class RpmMechanismIO:
    """
    Drive I/O for a mechanism that has an RPM Goal
    """
    @autolog
    @dataclass
    class RpmMechanismIOInputs:
        drive_connected: bool = False
        encoder_connected: bool = False

        drive_position: radians = 0.0  # rad
        drive_velocity: radians_per_second = 0.0  # rad / sec
        drive_applied: volts = 0.0  # volts
        drive_supply_current: amperes = 0.0  # amps

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: RpmMechanismIOInputs) -> None:
        """Update the drive I/O inputs.

        Args:
            inputs (RpmMechanismIOInputs): The drive I/O inputs to update.
        """
        pass

    def setVelocityGoal(self, goal: radians_per_second) -> None:
        """Set the drive.

        Args:
            goal (Radians/Sec): The desired RPM in radians per second.
        """
