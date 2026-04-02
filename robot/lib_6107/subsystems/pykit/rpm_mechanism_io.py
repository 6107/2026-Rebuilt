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

from wpimath.units import amperes, radians, radians_per_second, volts

from lib_6107.pykit.autolog import autolog


class RpmMechanismIO:
    """
    Drive I/O for a mechanism that has an RPM Goal
    """
    @autolog
    @dataclass
    class RpmMechanismIOInputs:
        mechanism_connected: bool = False

        mechanism_position: radians = 0.0
        mechanism_velocity: radians_per_second = 0.0
        mechanism_applied_voltage: volts = 0.0
        mechanism_supply_current: amperes = 0.0

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: RpmMechanismIOInputs) -> None:
        """Update the drive I/O inputs.

        Args:
            inputs (RpmMechanismIOInputs): The drive I/O inputs to update.
        """
        pass
