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

from commands2 import TimedCommandRobot
from pykit.logger import Logger
from wpilib import DSControlWord, RobotController


class LoggedTimedCommandRobot(TimedCommandRobot):
    """
    Provides a wpilib TimedCommandRobot with pykit logging capabilities.

    This class can be used as your MyRobot base if you need a Timed robot with
    both Commands2 and pykit (AdvantageScope) support.

    Since pykit's LoggedRobot does not support specifying the default period in
    the initializer, we will not support that either at this time.
    """
    kDefaultPeriod = TimedCommandRobot.kDefaultPeriod  # microseconds
    default_period = kDefaultPeriod / 1000  # seconds

    def __init__(self):
        """
        Constructor for the LoggedTimedCommandRobot.
        Initializes the robot, sets up the logger, and creates I/O objects.
        """
        self._greatest_period = self.getPeriod()
        self._greatest_offset = 0

        super().__init__(period=self.default_period)

        self._period_started = False

        # NOTE: When performing replay (RobotMode == REPLAY), set the following
        #       self.use_timing to False
        self.useTiming = True

        self.word = DSControlWord()
        self.user_code_start = 0
        self.periodic_before_start = 0
        self.init_end = None

    def robotInit(self) -> None:
        super().robotInit()
        self.pykit_startup_steps()

    def pykit_startup_steps(self) -> None:
        self.init_end = RobotController.getFPGATime()
        Logger.periodicAfterUser(self.init_end, 0)

        # Add our end-of-loop function to callbacks
        super().addPeriodic(self._end_of_loop_func, self._greatest_period,
                            self._greatest_offset + 1)

        # Technically, pykit calls hal.observeUserProgramStarting() here, but we need to
        # let the base class do that. Pykit then started the receiver which I guess we need
        # to do right now.
        Logger.startReciever()

    def addPeriodic(self, callback, *args, **kwargs):  # real signature unknown; NOTE: unreliably restored from __doc__
        """
        addPeriodic(self: wpilib._wpilib.TimedRobot, callback: collections.abc.Callable[[], None], period: wpimath.units.seconds, offset: wpimath.units.seconds = 0.0) -> None

        We override this class to track any callbacks that get added. We need to scheduler our own
        'end-of-loop' callback to run last

        :param callback: The callback to run.
        :param period:   The period at which to run the callback.
        :param offset:   The offset from the common starting time. This is useful
                         for scheduling a callback in a different timeslot relative
                         to TimedRobot.
        """
        arg_cnt = 0
        if 'period' in kwargs:
            period = kwargs['period']
        else:
            period = args[arg_cnt]
            arg_cnt += 1

        offset = kwargs['offset'] if 'offset' in kwargs else args[arg_cnt]

        if period >= self._greatest_period and offset > self._greatest_offset:
            self._greatest_period, self._greatest_offset = period, offset

        # Call into the base class to finish
        super().addPeriodic(callback, *args, **kwargs)

    #################################################################################
    #
    # For the '*Init', '*Periodic', and '*Exit' functions below, these may be called
    # from the IterativeRobot base. They could also be skipped.
    #
    # What I am trying to do here is enable the pykit logging as close as to the start
    # of a loop as the pykit's Logged robot does.
    #
    # If we can get the C++ library in iterative robot base to expose a function we
    # could attach to, then, we would not need all this 'mess'.
    #
    def disabledInit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().disabledInit()

    def disabledPeriodic(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().disabledPeriodic()

    def disabledExit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().disabledExit()

    def autonomousInit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().autonomousInit()

    def autonomousPeriodic(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().autonomousPeriodic()

    def autonomousExit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().autonomousExit()

    def teleopInit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().teleopInit()

    def teleopPeriodic(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().teleopPeriodic()

    def teleopExit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().teleopExit()

    def testInit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().testInit()

    def testPeriodic(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().testPeriodic()

    def testExit(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().testExit()

    def robotPeriodic(self):
        # The following insures we start up the logging for this next loop, but only once
        self._start_of_loop_func()
        super().robotPeriodic()

    def _start_of_loop_func(self):
        if not self._period_started:
            self._period_started = True
            self._start_of_loop_func()

            self.periodic_before_start = self.getLoopStartTime() or RobotController.getFPGATime()
            Logger.periodicBeforeUser()

            # Execute user periodic code and measure timing
            self.user_code_start = RobotController.getFPGATime()

    def _end_of_loop_func(self):
        """
        End of the period Loop function from the Iterative Robot base. This function is scheduled
        to be the last function called by the CommandScheduler, so we know that all subsystems have
        been updated an any new commands issued.
        """
        ###############################################
        # Now let's close out the pykit logging for this loop.
        #
        # Close out the iterative robot base iteration with logger
        user_code_end = RobotController.getFPGATime()

        # Run logger post-user code (save outputs to log)
        Logger.periodicAfterUser(user_code_end - self.user_code_start,
                                 self.user_code_start - self.periodic_before_start)
        self._period_started = False
