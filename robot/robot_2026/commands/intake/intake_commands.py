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

from typing import Optional

from commands2 import Subsystem
from pathplannerlib.auto import NamedCommands
from wpimath.units import revolutions_per_minute

from lib_6107.commands.command import BaseCommand


class IntakeCollectFuel(BaseCommand):  # change the name for your command
    """
    Activates the intake to collect fuel and deactivates it on end. The
    'speed' parameter provides the voltage rate for the intake.
    """
    name = "IntakeCollectFuel"

    def __init__(self, container, rpm: Optional[revolutions_per_minute] = 500):
        super().__init__(container)

        self._rpm: revolutions_per_minute = rpm
        self._intake: Subsystem = None                   # TODO: Define this once we have it

    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> IntakeCollectFuel:
            return IntakeCollectFuel(container, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(command().name, command())

    def initialize(self) -> None:
        """
        The initial subroutine of a command. Called once when the command is initially scheduled.
        """
        super().initialize()

        # TODO: Start the intake

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
        # TODO: Stop the intake

        super().end(interrupted)
