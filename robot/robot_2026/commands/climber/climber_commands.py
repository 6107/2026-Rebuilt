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

import logging

from pathplannerlib.auto import NamedCommands

from lib_6107.commands.command import BaseCommand
from lib_6107.util.elastic_utils import Notification, NotificationLevel, send_notification
from robot_2026.subsystems.rev_climber import ClimberConstants, RevClimber

logger = logging.getLogger(__name__)

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
    def __init__(self, container: 'RobotContainer', position: float):
        super().__init__(container)

        self._climber: RevClimber = container.climber
        self._position_goal: float = position
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
        self._running = False

    def execute(self) -> None:
        """
        The main body of a command. Called repeatedly while the command is scheduled.
        """
        if not self._running:
            self._running = True

            if self._climber.enabled:
                logger.info(f"{self.__class__.__name__}: execute")
                self._climber.position = self._position_goal
            else:
                send_notification(Notification(NotificationLevel.WARNING,
                                               title="Not Available",
                                               description="Climber Subsystem is DISABLED",
                                               display_time=1500))

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        return not self._climber.enabled

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        self._climber.stop()
        self._running = False

        super().end(interrupted)


class RetractClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will retract the climber (robot goes up). It
    can be attached to controller button or used by an automated routine.

    The 'RevClimber' subsystem used by this command should have a stall current
    set so if we fully retract the climbing arm, the stall current limit should
    trip.

    Retraction should go quickly, so a short (1.5 second?) timeout may be wise
    """

    def __init__(self, container: 'RobotContainer'):
        super().__init__(container, ClimberConstants.CLIMBER_RETRACTED_SETPOINT)

    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), RetractClimber(container))

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """

        super_finished = super().isFinished()
        climber_finished = self._climber.position <= self._position_goal + ClimberConstants.CLIMBER_TOLERANCE

        logger.info(f"Climber retract: super: {super_finished}, climber: {climber_finished}, "
                    f"pos: {self._climber.position}, goal: {self._position_goal}, toll: {ClimberConstants.CLIMBER_ROOT_X}")

        return super_finished or climber_finished

        # return super().isFinished() or \
        #     self._climber.position >= self._position_goal - ClimberConstants.CLIMBER_TOLERANCE


class ExtendClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will extend the climber (robot goes down). It
    can be attached to controller button or used by an automated routine.

    Extension is slower than retraction, so make sure you give 4-5 seconds before you
    need to use the Extended Climber.
    """
    def __init__(self, container: 'RobotContainer'):
        super().__init__(container, ClimberConstants.CLIMBER_EXTENDED_SETPOINT)

    @staticmethod
    def pathplanner_register(container: 'RobotContainer') -> None:
        """
        This command factory can be used with register this command
        and make it available from within PathPlanner
        """
        # Register the function itself
        NamedCommands.registerCommand(BaseCommand.get_class_name(), ExtendClimber(container))

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        super_finished = super().isFinished()
        climber_finished = self._climber.position >= self._position_goal + ClimberConstants.CLIMBER_TOLERANCE

        logger.info(f"Climber Extend: super: {super_finished}, climber: {climber_finished}, "
                    f"pos: {self._climber.position}, goal: {self._position_goal}, toll: {ClimberConstants.CLIMBER_ROOT_X}")

        return super_finished or climber_finished

class TweekUpClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will extend the climber an extra 1/4 inch (which)
    may be less depending on the current location.
    """

    def __init__(self, container: 'RobotContainer'):
        self._start_point = container.climber.position
        self._increment = -0.25

        super().__init__(container, self._start_point - self._increment)

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time. This assumes we are flat on the floor
        and in position to retract.
        """
        super().initialize()

        # Reset the climbing subsystem.
        self._running = False

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        super_finished = super().isFinished()
        climber_finished = self._climber.position >= self._position_goal

        logger.info(f"TweekUpClimber: super: {super_finished}, climber: {climber_finished}, "
                    f"pos: {self._climber.position}, goal: {self._position_goal}")

        return super().isFinished() or \
            self._climber.position >= self._position_goal


class TweekDownClimber(ClimberBaseCommand):  # change the name for your command
    """
    This command, while it runs, will retract the climber an extra 1/4 inch (which)
    may be less depending on the current location.
    """

    def __init__(self, container: 'RobotContainer'):
        self._start_point = container.climber.position
        self._increment = 0.25

        super().__init__(container, self._start_point + self._increment)

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time. This assumes we are flat on the floor
        and in position to retract.
        """
        logger.info(f"{self.__class__.__name__}: initialized")
        super().initialize()

        # Reset the climbing subsystem.
        self._running = False

    def isFinished(self) -> bool:
        """
        Whether the command has finished. Once a command finishes, the scheduler will call its :meth:`commands2.Command.end`
        method and un-schedule it.

        :returns: whether the command has finished.
        """
        super_finished = super().isFinished()
        climber_finished = self._climber.position <= self._position_goal

        logger.info(f"TweekDownClimber: super: {super_finished}, climber: {climber_finished}, "
                    f"pos: {self._climber.position}, goal: {self._position_goal}")

        return super().isFinished() or \
            self._climber.position <= self._position_goal
