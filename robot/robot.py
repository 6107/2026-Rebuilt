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
from wpilib import DriverStation, Field2d, LiveWindow, RobotBase, SmartDashboard, Timer
from wpimath.units import seconds

import constants
from constants import USE_PYKIT
from lib_6107.util.statistics import RobotStatistics
from robotcontainer import RobotContainer
from version import VERSION

if USE_PYKIT:
    # pykit & AdvantageScope support
    from pykit.wpilog.wpilogwriter import WPILOGWriter
    from pykit.wpilog.wpilogreader import WPILOGReader
    from pykit.networktables.nt4Publisher import NT4Publisher
    from pykit.logger import Logger
    from lib_6107.util.logged_timed_command_robot import LoggedTimedCommandRobot as MyRobotBase
else:
    from commands2 import TimedCommandRobot as MyRobotBase

# Setup Logging
logger = logging.getLogger(__name__)


def print_overrun_message(self):
    """Prints a message when the main loop overruns."""
    print("Loop overrun detected!")

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

        if USE_PYKIT:
            Logger.recordMetadata("Robot", type(self).__name__)
            Logger.recordMetadata("Team", "6107")
            Logger.recordMetadata("Year", "2026")

            match constants.ROBOT_MODE:
                case constants.RobotModes.REAL:
                    deploy_config = wpilib.deployinfo.getDeployData()

                    if deploy_config is not None:
                        Logger.recordMetadata("Deploy Host", deploy_config.get("deploy-host", ""))
                        Logger.recordMetadata("Deploy User", deploy_config.get("deploy-user", ""))
                        Logger.recordMetadata("Deploy Date", deploy_config.get("deploy-date", ""))
                        Logger.recordMetadata("Code Path", deploy_config.get("code-path", ""))
                        Logger.recordMetadata("Git Hash", deploy_config.get("git-hash", ""))
                        Logger.recordMetadata("Git Branch", deploy_config.get("git-branch", ""))
                        Logger.recordMetadata("Git Description", deploy_config.get("git-desc", ""))

                    Logger.addDataReciever(NT4Publisher(True))
                    Logger.addDataReciever(WPILOGWriter())

                case constants.RobotModes.SIMULATION:
                    Logger.addDataReciever(WPILOGWriter())
                    Logger.addDataReciever(NT4Publisher(True))

                case constants.RobotModes.REPLAY:
                    #
                    #  To run back a log file in replay mode, set the `LOG_PATH` environment variable
                    #  and then run in simulation.
                    #
                    #  An example is to run the following:
                    #
                    #    LOG_PATH=/path/to/log/file.wpilog robotpy --main src sim
                    #
                    self.useTiming = False  # Disable timing in replay mode, run as fast as possible

                    log_path = os.environ["LOG_PATH"]
                    log_path = os.path.abspath(log_path)

                    Logger.setReplaySource(WPILOGReader(log_path))
                    Logger.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))

            Logger.start()

        self._counter = 0  # Updated on each periodic call. Can be used to logging/smartdashboard updates

        self._container: Optional[RobotContainer] = None
        self._autonomous_command: Optional[Command] = None
        self._auto_end_started = False

        self.disabledTimer: Timer = Timer()

        self.field: Optional[wpilib.Field2d] = None
        self._stats: RobotStatistics = RobotStatistics()
        self._is_simulation = RobotBase.isSimulation()

        self._network_tables_instance = NetworkTableInstance.getDefault()

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
        super().robotInit()

        if USE_PYKIT:
            SignalLogger.enable_auto_logging(False)
            LiveWindow.disableAllTelemetry()
            # TODO: make period smaller
            CommandScheduler.getInstance().setPeriod(0.5)  # 1/2s period for command scheduler wathdoc

            command_count: dict[str, int] = {}

            def logCommandFunction(command: Command, active: bool) -> None:
                name = command.getName()
                count = command_count.get(name, 0) + (1 if active else -1)
                command_count[name] = count
                Logger.recordOutput(f"Commands/{name}", count > 0)

            scheduler = CommandScheduler.getInstance()

            scheduler.onCommandInitialize(lambda c: logCommandFunction(c, True))
            scheduler.onCommandFinish(lambda c: logCommandFunction(c, False))
            scheduler.onCommandInterrupt(lambda c: logCommandFunction(c, False))

        # Set up logging
        self._logging_init()

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

        logger.info("robotInit: exit")

    def _logging_init(self):
        match constants.ROBOT_MODE:
            case constants.RobotModes.REAL:
                logger.setLevel(logging.ERROR)  # Python logging
                logging.getLogger("wpilib").setLevel(logging.ERROR)
                logging.getLogger("commands2").setLevel(logging.ERROR)

            case constants.RobotModes.SIMULATION:
                DriverStation.silenceJoystickConnectionWarning(True)
                logger.setLevel(logging.INFO)  # Python logging
                logging.getLogger("wpilib").setLevel(logging.DEBUG)
                logging.getLogger("commands2").setLevel(logging.DEBUG)

            case constants.RobotModes.REPLAY:
                logger.setLevel(logging.ERROR)  # Python logging
                logging.getLogger("wpilib").setLevel(logging.ERROR)
                logging.getLogger("commands2").setLevel(logging.ERROR)

    def endCompetition(self):  # real signature unknown; restored from __doc__
        print("========================================")
        print("Robot Statistics:")
        self._stats.print("all", 1)
        print("========================================")
        # self._network_tables_instance.stopClient()

    def robotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver
        station. All classes derived from 'Subsystem' will have their 'periodic'
        function called automatically (right after this function). So only do
        non-Subsystem updates here

        Default period is 20 mS.
        """
        start = time.monotonic()
        super().robotPeriodic()

        # if USE_PYKIT and False:
        #     # This routine is called
        #     from util.logtracer import LogTracer
        #     from util.phoenixutil import PhoenixUtil
        #
        #     LogTracer.resetOuter("RobotPeriodic")
        #     PhoenixUtil.updateSignals()               TODO: this is an optimization we may want
        #     LogTracer.record("PhoenixUpdate")
        #
        #     self.container.robotPeriodic()
        #
        #     LogTracer.record("ContainerPeriodic")
        #     CommandScheduler.getInstance().run()
        #     LogTracer.record("CommandsPeriodic")
        #     LogTracer.recordTotal()

        self._counter += 1
        self._stats.add("periodic", time.monotonic() - start)

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
        logger.info("disabledInit: entry")
        super().disabledInit()

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
        super().disabledPeriodic()
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
        super().disabledExit()
        logger.info("*** disabledExit: entry")
        self.disabledTimer.stop()
        self.disabledTimer.reset()

    def autonomousInit(self) -> None:
        """
        Initialization code for autonomous mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters autonomous mode.
        """
        super().autonomousInit()
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
        super().autonomousPeriodic()
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
        super().autonomousExit()
        logger.info("autonomousExit: entry")

        if self._autonomous_command:
            self._autonomous_command.cancel()

    def teleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters teleop mode.
        """
        super().teleopInit()
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
        super().teleopPeriodic()
        start = time.monotonic()
        pass

        self._stats.add("teleop", time.monotonic() - start)

    def teleopExit(self) -> None:
        """
        Exit code for teleop mode should go here.

        Users should override this method for code which will be called each time
        the robot exits teleop mode.
        """
        super().teleopExit()
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
        super().testInit()
        logger.info("*** called testInit")
        CommandScheduler.getInstance().cancelAll()

    def testPeriodic(self):
        """
        Periodic code for test mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in test
        mode.
        """
        super().testPeriodic()
        logger.info("*** called testPeriodic")
        pass

    def testExit(self):
        """
        Exit code for test mode should go here.

        Users should override this method for code which will be called each time
        the robot exits test mode.
        """
        super().testExit()
        logger.info("*** called testExit")
        pass
