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

from wpimath.units import amperes, inches, volts

from lib_6107.pykit.autolog import autolog


class RotationMechanismIO:
    """
    Drive I/O for a mechanism that has number of rotations as a goal
    """
    @autolog
    @dataclass
    class RotationMechanismIOInputs:
        mechanism_connected: bool = False

        mechanism_position: inches = 0.0
        mechanism_speed: float = 0.0            # Inches / second
        mechanism_applied_voltage: volts = 0.0
        mechanism_supply_current: amperes = 0.0
        # mechanism_torque_amps: amperes = 0.0  # TODO: Figure this out or drop it

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: RotationMechanismIOInputs) -> None:
        """
        Update the drive I/O inputs.

        Args:
            inputs (RpmMechanismIOInputs): The drive I/O inputs to update.
        """
        pass

    def set_position(self, position: inches) -> None:
        """
        Set the drive

        Args:
            position (rotations +/-): The desired number of rotations
        """
        pass

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        pass