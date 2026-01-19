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
from typing import Optional

import wpilib
from commands2 import CommandScheduler
from commands2.command import Command
from wpilib import Timer, RobotBase, DriverStation, Field2d, SmartDashboard

import constants
from constants import USE_PYKIT
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
        self.autonomousCommand: Optional[Command] = None

        self.disabledTimer: Timer = Timer()
        self.autonomousCommand: Optional[Command] = None

        self.holding_alge = False
        self.autonomousCommand = None
        self.field: Optional[wpilib.Field2d] = None

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
        # Set up logging
        if USE_PYKIT:
            Logger.recordMetadata("Robot", "Team6107-2026")

            match constants.kRobotMode:
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

        # Set up our playing field. May get overwritten if simulation is running or if we
        # support vision based odometry
        self.field = Field2d()
        SmartDashboard.putData("Field", self.field)

        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        self._container = RobotContainer(self)
        self.disabledTimer = wpilib.Timer()

    def robotPeriodic(self) -> None:
        """
        Periodic code for all modes should go here.

        This function is called each time a new packet is received from the driver
        station. All classes derived from 'Sybsystem' will have their 'periodic'
        function called automatically (right after this function). So only do
        non-Subsystem updates here

        Default period is 20 mS.
        """
        logger.debug(f"called robotPeriodic: enabled: {self.isEnabled()}")
        self._counter += 1

    def disabledInit(self) -> None:
        """
        Initialization code for disabled mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters disabled mode.
        """
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
        logger.debug("called disabledPeriodic")

        if self.disabledTimer.hasElapsed(constants.WHEEL_LOCK_TIME):
            self.container.robot_drive.set_motor_brake(False)
            self.disabledTimer.stop()
            self.disabledTimer.reset()

        # Validate who we are working for
        if not self.match_started:
            self.container.check_alliance()

    def disabledExit(self) -> None:
        """
        Exit code for disabled mode should go here.

        Users should override this method for code which will be called each time
        the robot exits disabled mode.
        """
        logger.debug("*** disabledExit: entry")
        self.disabledTimer.stop()
        self.disabledTimer.reset()

    def autonomousInit(self) -> None:
        """
        Initialization code for autonomous mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters autonomous mode.
        """
        self.container.set_start_time()

        # Stop what we are doing...
        self.container.robot_drive.set_motor_brake(True)

        # Validate who we are working for. This may not be valid until autonomous or teleop init
        if not self.match_started:
            self.container.check_alliance()
            self.match_started = True

        self.autonomousCommand = self.container.getAutonomousCommand()

        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """
        Periodic code for autonomous mode should go here.

        Users should override this method for code which will be called each time a
        new packet is received from the driver station and the robot is in
        autonomous mode.
        """
        logger.debug("*** called autonomousPeriodic")

    def autonomousExit(self) -> None:
        """
        Exit code for autonomous mode should go here.

        Users should override this method for code which will be called each time
        the robot exits autonomous mode.
        """
        logger.info("*** autonomousExit: entry")

        if self.autonomousCommand:
            self.autonomousCommand.cancel()

    def teleopInit(self) -> None:
        """
        Initialization code for teleop mode should go here.

        Users should override this method for initialization code which will be
        called each time the robot enters teleop mode.
        """
        logger.debug("*** called teleopInit")

        self.container.set_start_time()

        # Stop what we are doing...
        if self.autonomousCommand:
            self.autonomousCommand.cancel()
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
        # Intake wheel control logic
        logger.debug("*** called teleopPeriodic")

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
