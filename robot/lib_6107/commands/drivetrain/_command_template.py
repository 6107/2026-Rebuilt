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
#  Drivetrain specific Command template
#

from pathplannerlib.auto import NamedCommands

from lib_6107.commands.command import BaseCommand


class DriveTrainCommandTemplate(BaseCommand):  # change the name for your command
    """
    TODO: Describe this class here
    """
    def __init__(self, drivetrain: 'DriveSubsystem',  **_kwargs):
        super().__init__(drivetrain)

        raise NotImplementedError("Remember to remove this line as well")

    @staticmethod
    def pathplanner_register(drivetrain: 'DriveSubsystem') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        def command(**kwargs) -> DriveTrainCommandTemplate:
            return DriveTrainCommandTemplate(drivetrain, **kwargs)      # TODO: Rename this too

        # Register the function itself
        NamedCommands.registerCommand(command().name, command())

    def initialize(self) -> None:
        """
        The initial subroutine of a command. Called once when the command is initially scheduled.
        """
        super().initialize()

        pass

    def execute(self) -> None:
        """
        The initial subroutine of a command. Called once when the command is initially scheduled.
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
