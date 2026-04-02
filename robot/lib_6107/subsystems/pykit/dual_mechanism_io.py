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

from wpimath.units import amperes, degrees, inches, volts

from lib_6107.pykit.autolog import autolog


class DualMechanismIO:
    """
    Drive I/O for a mechanism that has two motors working in tandem
    """
    @autolog
    @dataclass
    class DualMechanismIOInputs:
        mechanism_1_connected: bool = False
        mechanism_2_connected: bool = False

        mechanism_1_position: inches | degrees = 0.0
        mechanism_1_speed: float = 0.0            # Inches / second
        mechanism_1_applied_voltage: volts = 0.0
        mechanism_1_supply_current: amperes = 0.0
        mechanism_1_torque_amps: amperes = 0.0

        mechanism_2_position: inches | degrees = 0.0
        mechanism_2_speed: float = 0.0            # Inches / second
        mechanism_2_applied_voltage: volts = 0.0
        mechanism_2_supply_current: amperes = 0.0
        mechanism_2_torque_amps: amperes = 0.0

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: DualMechanismIOInputs) -> None:
        """
        Update the drive I/O inputs.

        Args:
            inputs (RpmMechanismIOInputs): The drive I/O inputs to update.
        """
        pass

    def set_position(self, device: int) -> None:
        """
        Set the drive

        Args:
            device (int): Device number 1 or 2
        """
        pass

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        pass