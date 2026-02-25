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
#  It provides a starting place to define a Command. Just copy it to a new
#  filename and change the class name and implementation to suite what you
#  may need in your project.
#

from commands2 import Command
from pathplannerlib.auto import NamedCommands

from lib_6107.commands.command import BaseCommand


class CommandTemplate(BaseCommand):  # change the name for your command
    """
    TODO: Describe this class here
    """
    def __init__(self, container: 'RobotContainer',  **_kwargs):
        super().__init__(container)

        raise NotImplementedError("Remember to remove this line as well")

    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> Command:
            return CommandTemplate(container, **kwargs)      # TODO: Rename this too

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), command())

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time
        """
        super().initialize()

        pass


    def execute(self) -> None:
        """
        The main body of a command. Called repeatedly while the command is scheduled.
        """
        pass

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return True

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        pass

        super().end(interrupted)