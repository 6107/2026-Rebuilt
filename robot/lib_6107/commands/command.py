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

import logging
from typing import Optional

from commands2 import Command, Subsystem
from wpilib import RobotBase, SmartDashboard

logger = logging.getLogger(__name__)

class BaseCommand(Command):
    """
    Base Command class for Team 6107 robotics
    """
    def __init__(self, target):
        super().__init__()
        self.setName(self.get_class_name())

        from robotcontainer import RobotContainer
        from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem

        if not isinstance(target, (RobotContainer, DriveSubsystem)):
            raise ValueError(f"target must be a subclass of RobotContainer or DriveSubsystem")

        self._target: DriveSubsystem | RobotContainer = target
        self._drivetrain: Optional[DriveSubsystem] = None
        self._container: Optional[RobotContainer] = None

        if isinstance(self._target, Subsystem):
            self.addRequirements(target)  # commandsv2 version of requirements'

        self._start_time: float = 0
        self._log_level = logging.INFO if RobotBase.isSimulation() else logging.DEBUG

    @classmethod
    def get_class_name(cls) -> str:
        return cls.__name__

    def initialize(self) -> None:
        """
        Called just before this Command runs the first time
        """
        from robotcontainer import RobotContainer
        from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem

        if isinstance(self._target, RobotContainer):
            self._container: RobotContainer = self._target
            self._drivetrain: DriveSubsystem = self._target.robot_drive

        elif isinstance(self._target, DriveSubsystem):
            self._drivetrain: DriveSubsystem = self._target
            self._container: RobotContainer = self._target.container

        self._start_time = round(self._container.get_elapsed_time(), 2)
        logging.info(f"{self.getName()}: Started at {self._start_time}")

        SmartDashboard.putString(f"command/{self.getName()}", "running")
        SmartDashboard.putString("alert",
                                 f"** Started {self.getName()} at {self._start_time - self._container.get_elapsed_time():2.2f} s **")

    def end(self, interrupted: bool) -> None:
        """
        The action to take when the command ends. Called when either the command finishes normally, or
        when it interrupted/canceled.

        Do not schedule commands here that share requirements with this command. Use :meth:`.andThen` instead.

        :param interrupted: whether the command was interrupted/canceled
        """
        end_time = self._container.get_elapsed_time()
        message = f"{self.getName()}: {'Interrupted' if interrupted else 'Ended'} at {end_time:.1f} s after {end_time - self._start_time:.1f} s"
        logging.info(message)

        SmartDashboard.putString(f"alert",f"** {message} **")
        SmartDashboard.putString(f"command/{self.getName()}", f"{'interrupted' if interrupted else 'ended'}")
