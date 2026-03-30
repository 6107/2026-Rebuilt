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
#
# Autonomous climbing command.
#
from typing import Optional

import commands2
from pathplannerlib.auto import NamedCommands

from lib_6107.commands.command import BaseCommand
from robot_2026.commands.climber.climber_commands import RetractClimber


class AutoClimberSequence(commands2.SequentialCommandGroup):
    """
    For 2026 Rebuilt, we will have three possible climbing PathPlanner sequences'
    where we approach the ladder and climb. The climbing locations are C1, C2 and C3.

    For both C2 and C3, we are approaching the ladder, and it will be to our right.

    For C1, we have to go toward the wall and then, once again, move forward to our right.

    This sequence is called from all three of those. It will move a couple more inches
    to the right and then retract the climber.

    The 'extend climber command' is expected to already have extended the climber
    fully before this command is called.
    """

    def __init__(self, container, indent: Optional[int] = 0) -> None:
        super().__init__()

        self.setName("AutoClimberSequence")

        # ArcadeDrive...
        self.addCommands(commands2.PrintCommand(
            f"{'    ' * indent}** Started {self.getName()} This should be the inch right command**"))

        # ArcadeDrive...
        self.addCommands(commands2.PrintCommand(
            f"{'    ' * indent}** Started {self.getName()} This should be the inch forward command**"))

        # And pull the trigger
        self.addCommands(RetractClimber(container))

    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), AutoClimberSequence(container))
