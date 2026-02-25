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
#  This file originated from aesatchien's FRC2429_2025 project on github:
#       https://github.com/aesatchien/FRC2429_2025
#
#  It provides a starting place to define a Command Group. Just copy it to
#  a new filename and change the class name and implementation to suite what you
#  may need in your project.
#
from typing import Optional

import commands2


class CommandGroupTemplate(commands2.SequentialCommandGroup):
    """
    TODO: Describe this class here
    """
    def __init__(self, container, indent: Optional[int]=0) -> None:
        super().__init__()

        self.setName("Command Group Template- change me!")
        self.container = container
        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Started {self.getName()} **"))

        # self.addCommands(... your stuff here, call other commands with indent=indent+1 ...)

        self.addCommands(commands2.PrintCommand(f"{'    ' * indent}** Finished {self.getName()} **"))

        raise NotImplementedError("Remember to remove this line as well")
