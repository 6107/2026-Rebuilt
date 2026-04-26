#!/usr/bin/env python3
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
import os
import sys
import time
from typing import Optional

import wpilib
from commands2 import CommandScheduler
from commands2.command import Command
from ntcore import NetworkTableInstance
from pathplannerlib.pathfinding import LocalADStar, Pathfinding
from phoenix6 import SignalLogger
from rev import StatusLogger
from wpilib import DriverStation, Field2d, LiveWindow, SmartDashboard, Timer
from wpimath.units import seconds

from constants import MyRobotConstants
from robot_2026.util.logtracer import LogTracer
from robotcontainer import MyRobotContainer

from lib_6107.constants import RobotModes, ROBOT_MODE
from lib_6107.robotcontainer import RobotContainer
from lib_6107.pykit.logger import Logger
from lib_6107.pykit.loggedrobot import LoggedRobot
from lib_6107.pykit.networktables.nt4Publisher import NT4Publisher
from lib_6107.pykit.wpilog.wpilogreader import WPILOGReader
from lib_6107.pykit.wpilog.wpilogwriter import WPILOGWriter
from lib_6107.robot import Robot
from lib_6107.util.elastic_utils import Notification, select_tab, send_notification
from lib_6107.util.phoenix6_signals import Phoenix6Signals
from lib_6107.util.statistics import RobotStatistics

# Setup Logging
logger = logging.getLogger(__name__)

"""
The VM is configured to automatically run this class, and to call the functions corresponding to
each mode, as described in the TimedRobot documentation. If you change the name of this class or
the package after creating this project, you must also update the build.gradle file in the
project.
"""


class MyRobot(Robot):
    """
    Our default robot class

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def __init__(self):
        # Initialize our base class. Eventually creates a 'LoggedRobot' with CommandScheduler
        # support
        super().__init__("2026",
                         robot_constants=MyRobotConstants())

        # NOTE: A good bit of AdvantageScope, python logging, and other common subsystems
        #       have been set up at this point.
        #
        # Now for 2026-Rebuilt specific additions
        self._auto_end_started = False

    # @tracer.start_as_current_span("robotInit")
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code. For 2026, the base class method handles pretty much
        everything we need. The main exception is what our Robot Container is that sets
        up and centralizes all of our subsystems and mechanism
        """
        super().robotInit(MyRobotContainer.create)

    def endCompetition(self) -> None:
        """
        This function is called at the end of competition to clean up and exit. The
        base class once again handles all we need this year.
        """
        # NOTE: If anything year specific needs to be done, do it here before calling
        #       the base class
        pass

        # That's All Folks...
        super().endCompetition()

    def autonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode should go here.

        For 2026, we had an option specific for running near the end
        of the autonomous mode. So check for that before continuing into
        the base class for other autonomous support
        """
        if not self._auto_end_started:
            remaining: seconds = DriverStation.getMatchTime()

            if 0 < remaining <= self.robot_constants.AUTONOMOUS_END_TRIGGER_TIME:
                self._auto_end_started = True
                end_command = self.container.get_autonomous_end_game_command()

                if end_command is not None:
                    logger.info(f"triggering autonomous end command: {end_command.getName}, {remaining} seconds left")

                    if self._autonomous_command is not None:
                        self._autonomous_command.cancel()

                    self._autonomous_command = end_command

                    # Run it
                    end_command.schedule()

        super().autonomousPeriodic()
