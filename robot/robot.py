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
from networktables import NetworkTables
from pathplannerlib.pathfinding import Pathfinding, LocalADStar
from wpilib import Timer, RobotBase, DriverStation, Field2d, SmartDashboard
from wpimath.units import seconds

import constants
from constants import USE_PYKIT
from lib_6107.util.statistics import RobotStatistics
from robotcontainer import RobotContainer
# from lib_6107.timedcommandloggedrobot import TimedCommandLoggedRobot
# from util.telemetry import Telemetry
from version import VERSION

if USE_PYKIT:
    # pykit & AdvantageScope support
    from pykit.wpilog.wpilogwriter import WPILOGWriter
    from pykit.wpilog.wpilogreader import WPILOGReader
    from pykit.networktables.nt4Publisher import NT4Publisher
    from pykit.loggedrobot import LoggedRobot as MyRobotBase
    from pykit.logger import Logger
else:
    from commands2 import TimedCommandRobot as MyRobotBase

from phoenix6 import HootAutoReplay

# Setup Logging
logger = logging.getLogger(__name__)

"""
The VM is configured to automatically run this class, and to call the functions corresponding to
each mode, as described in the TimedRobot documentation. If you change the name of this class or
the package after creating this project, you must also update the build.gradle file in the
project.
"""
class MyRobot(MyRobotBase):
    """
    Our default robot class

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """
    def __init__(self):
        # Initialize our base class, choosing the default scheduler period
        super().__init__()

        self._counter = 0  # Updated on each periodic call. Can be used to logging/smartdashboard updates

        self._container: Optional[RobotContainer] = None
        self._autonomous_command: Optional[Command] = None
        self._auto_end_started = False
        self._time_and_joystick_replay = None

        self.disabledTimer: Timer = Timer()

        self.field: Optional[wpilib.Field2d] = None
        self._stats: RobotStatistics = RobotStatistics()
        self._network_tables_server = None  # Simulation only

        # Visualization and pose support
        self.match_started = False  # Set true on Autonomous or Teleop init

    @property
    def container(self) -> RobotContainer:
        return self._container

    @property
    def counter(self) -> int:
        return self._counter

    # @tracer.start_as_current_span("robotInit")
    def robotInit(self) -> None:
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        logger.info("robotInit: entry")

        if RobotBase.isSimulation():
            self._network_tables_server = NetworkTables.initialize(server="127.0.0.1")

        # Set up logging
        if USE_PYKIT:
            Logger.recordMetadata("Robot", "Team6107-2026")

            match constants.ROBOT_MODE:
                case constants.RobotModes.REAL:
                    # TODO: logger.setLevel(logging.ERROR)
                    deploy_config = wpilib.deployinfo.getDeployData()
                    if deploy_config is not None:
                        Logger.recordMetadata("Deploy Host",
                                              deploy_config.get("deploy-host", ""))
                        Logger.recordMetadata("Deploy User",
                                              deploy_config.get("deploy-user", ""))
                        Logger.recordMetadata("Deploy Date",
                                              deploy_config.get("deploy-date", ""))
                        Logger.recordMetadata("Code Path",
                                              deploy_config.get("code-path", ""))
                        Logger.recordMetadata("Git Hash",
                                              deploy_config.get("git-hash", ""))
                        Logger.recordMetadata("Git Branch",
                                              deploy_config.get("git-branch", ""))
                        Logger.recordMetadata("Git Description",
                                              deploy_config.get("git-desc", ""))
                    Logger.addDataReciever(NT4Publisher(True))
                    Logger.addDataReciever(WPILOGWriter())

                case constants.RobotModes.SIMULATION:
                    # TODO: logger.setLevel(logging.INFO)
                    Logger.addDataReciever(NT4Publisher(True))
                    DriverStation.silenceJoystickConnectionWarning(True)

                case constants.RobotModes.REPLAY:
                    self.useTiming = False  # run as fast as possible
                    log_path = os.environ["LOG_PATH"]
                    log_path = os.path.abspath(log_path)

                    print(f"Starting log from {log_path}")

                    Logger.setReplaySource(WPILOGReader(log_path))
                    Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))

            Logger.start()
        else:
            if RobotBase.isSimulation():
                logger.setLevel(logging.INFO)

                # If this is a simulation, we need to silence joystick warnings
                logger.warning("Simulation detected. Silencing annoying JoyStick warnings")
                DriverStation.silenceJoystickConnectionWarning(True)
            else:
                logger.setLevel(logging.ERROR)

            logging.getLogger("wpilib").setLevel(logging.DEBUG)
            logging.getLogger("commands2").setLevel(logging.DEBUG)

            version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
            logger.info(f"Python: {version}, Software Version: {VERSION}")

        # Set up our pathfinding algorithm
        # TODO: LocalADStar has a dynamic obstacle field.
        #       Can we use that in future with vision?
        Pathfinding.setPathfinder(LocalADStar())

        # Set up our playing field. May get overwritten if simulation is running or if we
        # support vision based odometry
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer(self)
        self.disabledTimer = wpilib.Timer()

        # log and replay timestamp and joystick data
        self._time_and_joystick_replay = HootAutoReplay().with_timestamp_replay().with_joystick_replay()

        logger.info("robotInit: exit")

    def endCompetition(self):  # real signature unknown; restored from __doc__
        """
        endCompetition(self: wpilib._wpilib.TimedRobot) -> None

        Ends the main loop in StartCompetition().
        """
        print("========================================")
        print("Robot Statistics:")
        self._stats.print("all", 1)
        print("========================================")

    def robotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver
        station. All classes derived from 'Sybsystem' will have their 'periodic'
        function called automatically (right after this function). So only do
        non-Subsystem updates here

        Default period is 20 mS.
        """
        start = time.monotonic()
        self._time_and_joystick_replay.update()

        self._counter += 1
        self._stats.add("periodic", time.monotonic() - start)

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
        logger.info("disabledInit: entry")

        for subsystem in self.container.subsystems:
            if hasattr(subsystem, "stop") and callable(getattr(subsystem, "stop")):
                subsystem.stop()

        self.container.disablePIDSubsystems()

        self.disabledTimer.reset()
        self.disabledTimer.start()

    def disabledPeriodic(self) -> None:
        """
        Periodic code for disabled mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in disabled
        mode.
        """
        start = time.monotonic()
        logger.debug("called disabledPeriodic")

        if self.disabledTimer.hasElapsed(constants.WHEEL_LOCK_TIME):
            self.container.robot_drive.set_motor_brake(False)
            self.disabledTimer.stop()
            self.disabledTimer.reset()

        # Validate who we are working for
        if not self.match_started:
            self.container.check_alliance()

        self._stats.add("disabled", time.monotonic() - start)

    def disabledExit(self) -> None:
        """
        Exit code for disabled mode should go here.

        Users should override this method for code which will be called each time
        the robot exits disabled mode.
        """
        logger.info("*** disabledExit: entry")
        self.disabledTimer.stop()
        self.disabledTimer.reset()

    def autonomousInit(self) -> None:
        """
        Initialization code for autonomous mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters autonomous mode.
        """
        logger.info("autonomousInit: entry")

        self.container.set_start_time()

        # Stop what we are doing...
        self.container.robot_drive.set_motor_brake(True)

        # Validate who we are working for. This may not be valid until autonomous or teleop init
        if not self.match_started:
            self.container.check_alliance()
            self.match_started = True

        self._auto_end_started = False
        self._autonomous_command = self.container.get_autonomous_command()

        if self._autonomous_command:
            self._autonomous_command.schedule()

    def autonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in
        autonomous mode.
        """
        start = time.monotonic()

        if not self._auto_end_started:
            remaining: seconds = DriverStation.getMatchTime()

            if 0 < remaining <= constants.AUTONOMOUS_END_TRIGGER_TIME:
                self._auto_end_started = True
                end_command = self.container.get_autonomous_end_game_command()

                if end_command is not None:
                    logger.info(f"triggering autonomous end command: {end_command.getName}, {remaining} seconds left")

                    self._autonomous_command.cancel()
                    self._autonomous_command = end_command
                    # Run it
                    end_command.schedule()

            self._stats.add("auto", time.monotonic() - start)

    def autonomousExit(self) -> None:
        """
        Exit code for autonomous mode should go here.

        Users should override this method for code which will be called each time
        the robot exits autonomous mode.
        """
        logger.info("autonomousExit: entry")

        if self._autonomous_command:
            self._autonomous_command.cancel()

    def teleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters teleop mode.
        """
        logger.debug("*** called teleopInit")

        self.container.set_start_time()

        # Stop what we are doing...
        if self._autonomous_command:
            self._autonomous_command.cancel()
        else:
            CommandScheduler.getInstance().cancelAll()

        # Validate who we are working for. This may not be valid until autonomous or teleop init
        if not self.match_started:
            self.container.check_alliance()
            self.match_started = True

    def teleopPeriodic(self) -> None:
        """
        Periodic code for teleop mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in teleop
        mode.
        """
        start = time.monotonic()
        pass

        self._stats.add("teleop", time.monotonic() - start)

    def teleopExit(self) -> None:
        """
        Exit code for teleop mode should go here.

        Users should override this method for code which will be called each time
        the robot exits teleop mode.
        """
        for subsystem in self.container.subsystems:
            if hasattr(subsystem, "stop") and callable(getattr(subsystem, "stop")):
                subsystem.stop()

            self.container.robot_drive.set_straight()

    def testInit(self) -> None:
        """
        Initialization code for test mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters test mode.
        """
        logger.info("*** called testInit")
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """
        Periodic code for test mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in test
        mode.
        """
        logger.info("*** called testPeriodic")
        pass

    def testExit(self):
        """
        Exit code for test mode should go here.

        Users should override this method for code which will be called each time
        the robot exits test mode.
        """
        logger.info("*** called testExit")
        pass

    def _simulationInit(self) -> None:
        """
        Robot-wide simulation initialization code should go here.

        Users should override this method for default Robot-wide simulation
        related initialization which will be called when the robot is first
        started. It will be called exactly one time after RobotInit is called
        only when the robot is in simulation.
        """
        logger.info("*** _simulationInit: entry")

    def _simulationPeriodic(self):
        """
        Periodic simulation code should go here.

        This function is called in a simulated robot after user code executes.
        """
        logger.info("*** _simulationPeriodic: entry")
