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
from robot_2026.subsystems.rev_climber import RevClimber as Climber

RETRACT_LEVEL_ONE_REVOLUTIONS = 5
EXTEND_LEVEL_ONE_REVOLUTIONS = -(RETRACT_LEVEL_ONE_REVOLUTIONS + 1)


class ClimberBaseCommand(BaseCommand):  # change the name for your command
    """
    This command, while it runs, will retract the climber (robot goes up). It
    can be attached to controller button or used by an automated routine.

    The 'RevClimber' subsystem used by this command should have a stall current
    set so if we fully retract the climbing arm, the stall current limit should
    trip.
    """

    def __init__(self, container: 'RobotContainer', **kwargs):
        super().__init__(container)

        self._climber: Climber = container.climber
        self._position_goal: float = kwargs.get("position_goal", 0.0)
        self._manual_command: bool = kwargs.get("manual", False)
        self._running = False

        # This command needs to reserve the climbing subsystem
        self.addRequirements(self._climber)

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time. This assumes we are flat on the floor
        and in position to retract.
        """
        super().initialize()

        # Reset the climbing subsystem.
        self._climber.reset()
        self._running = False

        # TODO: Anything to turn of an existing brake request

    def execute(self) -> None:
        """
        The main body of a command. Called repeatedly while the command is scheduled.
        """
        if not self._running:
            self._running = True
            self._climber.position = self._position_goal

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        self._climber.stop(True)
        self._running = False

        super().end(interrupted)


class RetractClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will retract the climber (robot goes up). It
    can be attached to controller button or used by an automated routine.

    The 'RevClimber' subsystem used by this command should have a stall current
    set so if we fully retract the climbing arm, the stall current limit should
    trip.
    """
    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """

        def command(**kwargs) -> Command:
            return RetractClimber(container, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(),
                                      command(position_goal=RETRACT_LEVEL_ONE_REVOLUTIONS))

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return self._climber.position >= self._position_goal and not self._manual_command


class ExtendClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will extend the climber (robot goes down). It
    can be attached to controller button or used by an automated routine.
    """
    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """

        def command(**kwargs) -> Command:
            return RetractClimber(container, **kwargs)

        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(),
                                      command(position_goal=EXTEND_LEVEL_ONE_REVOLUTIONS))

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return self._climber.position <= self._position_goal and not self._manual_command
