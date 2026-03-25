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
from wpimath.units import amperes, revolutions_per_minute, volts


class RpmMechanismIO:
    """
    Drive I/O for a mechanism that has an RPM Goal
    """
    @autolog
    @dataclass
    class RpmMechanismIOInputs:
        drive_connected: bool = False
        encoder_connected: bool = False

        drive_velocity: revolutions_per_minute = 0.0
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

    def setVelocityGoal(self, goal: revolutions_per_minute) -> None:
        """Set the drive.

        Args:
            goal (Radians/Sec): The desired RPM in radians per second.
        """
