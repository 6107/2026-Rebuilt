# LoggedTimedCommandRobot class

The [LoggedTimedCommandRobot](#loggedtimedcommandrobot) class is an implementation of the **pykit**
_LoggedRobot_ class that also support the functionality of the **FRC**'s _TimedCommandRobot_ class.
For our [2026 Rebuilt Robot](https://github.com/6107/2026-Rebuilt), we had chosen to start with the
_TimedCommandRobot_ class due to heavy use of **Commandv2** _Commands_. However, we also wanted
to support 3d-simulation and log review/analysis provided by the [AdvantageScope](https://docs.advantagescope.org/)
application. This required us to build upon the existing [LoggedRobot](#loggedrobot) class.

This readme examines the [LoggedRobot](#loggedrobot) and [TimedCommandRobot](#timedcommandrobot) classes
in detail in order to determine how best to create the [LoggedTimedCommand](#loggedtimedcommandrobot) class and
any support classes.

# LoggedRobot

_LoggedRobot_ is provided by the pykit module to provided _AdvantageScope_ logging support for **FRC**
python implementations. Before we examine what this class provides, lets start at the very
lowest base class [RobotBase](#robotbase) and work our way up to the _LoggedRobot_'s base class
[IterativeRobotBase](#iterativerobotbase). Then we will get into the functionality that the
_LoggedRobot_ object provides.

## RobotBase

[RobotBase](https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/RobotBase.html#wpilib.RobotBase) is a
wrapper of the RobotBase C++ class from the [allwpilib](https://github.com/wpilibsuite/allwpilib) project on github.

This base class provides several useful utility functions (IsTest, IsTeleop, IsAutonomous, ...) and does nothing of
great interest that needs to be taken into account in this design. Two items of interest is the _StartCompetition_
and _EndCompetition_ functions are virtual functions.

However, in the RobotBase.h header file, there is a template function _StartRobot_ that is called in the **Robot.cpp**
_main_ function that starts up our robot and calls the _HAL_RunMain()_ function. These functions and related templates
call primarioly into the _StartCompetition_ and _EndCompetition_ functions. During debugging of our python robot
function, it became aware that unde the umbrella of _StartCompetition_ is really where all the hard work begins.

The **robotpy** python implementation of _RobotBase_ is found in the wpilib module and is a code-generated python
wrapper around the C++ class. All functions there contains no useful code and instructs the user to derive'
a subclass not from _RobotBase_ but either _IterativeRobotBase_ or _TimedRobot_. _IterativeRobotBase_ is the base
class for the **pykit** LoggedRobot and also an ancestor of the _TimedCommandRobot_ that we are wanting to
support.

## IterativeRobotBase

[IterativeRobotBase](https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/IterativeRobotBase.html)
is the base class of [LoggedRobot](#loggedrobot) presented above and is a descendant of the _RobotBase_ just
presented.

Like _RobotBase_, it is a code-generated wrapper around a C++ class implemented in
the [allwpilib](https://github.com/wpilibsuite/allwpilib)
project on github. The C++ class provides all the functions described for the RobotBase along with some additional
functions. The vast majority of these functions are empty and provide locations for people deriving their Robot
from this calls an orderly place to implement their custom robot code.

The main function of interest is the 'LoopFunc' which is exposed in the python generated code as __loopFunc_.
_IterativeRobotBase_ does not implement a _StartCompetition()_ function, so it should not be used directly by
a team's robot. This is where the __loopFunc()_ function in **pykit**'s _LoggedRobot_ and it support of both
_StartCompetition_ and _EndCompetition_ makes sense. **pykit** has full control of the robot an any periodic
calls into user code. We will look at that a little later in the [LoggedRobot](#loggedrobot-added-functionality) added
funcitonality section. But first, a review of what _LoopFunc_ does in the C++ code.

### LoopFunc

_LoopFunc()_ is heart of the periodic nature of the robot (default 20 mS period). It tracks what mode
(Teleop, Test, Autonomous, Disabled) the robot is in and what mode it was in on at the end of the last
cycle.

```text
    # Step - 1
    if (last-mode != mode):
        call the 'ModeExit' function for the last-mode  (TelopExit, DisabledExit, ...)

        call the 'ModeInit' function for the current mode  (TeleopInit, ...)

        set last-mode = mode
        
    # Step - 2
    call the 'mode-periodic' function for the current mode  (TeleopPeriodic, AutonomousPeriodic, ...)
    
    # Step - 3
    call the 'RobotPeriodic' function       (which is implemented in the final derived Robot object for a team)
    
    # Step - 4
    call SmartDashboard::UpdateValues()
    call LiveWindow::UpdateValues()
    call ShuffleBoard::Update()
    
    # Step - 5
    if simulation:
        call HAL_SimPeriodicBefore()    # Internal
        call SimulationPeriod           (which can be implemented in the final derived Robot object for a team)
        call HAL_SimPeriodicAfter()     # Internal
        
    # Step - 6
    Disable Watchdog
    Flush network tables if requested
    Check if watchdog had expired
    If expired:
        print watchdog Epochs           # Tracks what 'modeInit' and 'modePeriodic' functions called to helps
                                        # the developer look for what may be causing a period overrun.
    .. and then exit.
```

So the above information is the key to understanding exactly when something in a Team's robot is called.

## LoggedRobot added functionality

So, lets now look at what the **pykit** _LoggedRobot_ class provides us. Since we know we probably will not want
to derive or _LoggedTimedCommandRobot_ class with multiple base classes, knowing what it does an implementing a
class based off of _TimedCommandRobot_ that has its functionality intact may be the key.

Prior to \__init__, the default robot period is set to 20 mS (same default as all other **robotpy** Robot classes).

### \__init__

The \__init__ function first calls into the _IterativeRobotBase_ \__init__ method, passing in the default period. It
then creates the following attributes:

|    Attribute | Initial Value/Type | Description  (Units)                                                                            |
|-------------:|:------------------:|-------------------------------------------------------------------------------------------------|
|    useTiming |        True        | xxx                                                                                             | 
|    _periodUs | int(period * 1e6)  | Integer number of microseconds in the period (microseconds)                                     | 
| _nextCycleUs |   0 + _periodUs    | The microsecond start time for the next 'loop' to run                                           | 
|     notifier |   hal Notifier()   | A notifier is an FPGA controller timer that triggers at requested intervals based on FPGA time. | 
|     watchdog |     Watchdog()     | Watches for the period to expire. Prints message if it does.                                    | 
|         word |  DSControlWord()   | The current Driver Station control word.                                                        | 

### startCompletion

This is the heart of the _LoggedRobot_ and is called very soon after the robot's code is started up. It does the
following:

```text
    # Step - 1
    call robotInit()                # Calls into the team's MyRobot.robotInit() function
    
    if simulation:
        call _simulationInit()      # Calls into C++ code which eventually calls the PhysicsInterface
                                    # python __init__() function of the Team's simulated code (if implemented)
    # Step - 2
    set  initEnd attribute to the FPGA Time
    call Logger.periodicAfterUser(initEnd, 0)   # This finalizes the log entry for the current cycle. Since it is
                                                # being called before any loops have started, my assumption is it
                                                # will initialize the logger code in pykit for us.
    # Step - 3
    call hal.observeUserProgramStarting()  # Sets a flag for DriveStation to let it know the robot code is ready.
    
    # Step - 4
    call Logger.StartReceiver()   # Starts up all dataReceivers for the logger. These are populated by the Team's
                                  # robot code with the Logger.AddDataReceiver() function. In our team's robotInit(),
                                  # we have added receivers for:
                                  #   NT4Publisher, WPILOGWriter  -> if running for real
                                  #   NT4Publisher                -> if running in simulation
                                  #   WPILOGWriter                -> if running in replay mode
    # Step - 5
    Now start the big 'forever' loop to run our robot
        
        These steps detailed below
```

All the next steps are within the big 'forever' loop that runs the robot.

```text
This is a big 'while True' looo:

    # Step - 5a
    if 'useTiming' is True:
        get current time from FPGA
        
        if '_nextCycleUs' < current time:              # NOTE: Should this not be <=?
            Last cycle was > period. lets run the next loop right away  
            '_nextCycleUs' = current time
            
        else:
            set up hal UpdateNotifierAlarm to fire at '_nextCycleUs'
            wait for it to fire                     # NOTE: So we are blocking here
            
            if error from wait:
                Throw RuntimeError exception
                
            if returns 0 :
                break out of loop (normally returns the time we set it for)

        increment '_nextCycleUs' by '_periodUs'

    # Step - 5b
        set 'periodicBeforeStart' to current FPGA time

    # Step - 5c
        call Logger.periodicBeforeUser()    # updated the log for this new cycle through the code

    # Step - 5d
        set 'userCodeStart' to current FPGA time
        
            call _loopfunc()                # Run all those steps in LoopFunc section above

        set 'userCodeEnd' to current FPGA time

    # Step - 5e
        call Logger.periodicAfterUser with ('userCodeEnd'- 'userCodeStart'  and  'userCodeStart' - 'periodicBeforeStart'

    and then go back to the top of the loop.
 ```

### endCompletion

Calls the hal to stop the Notifier and clean the Notifier

### printOverrunMessage

Called by the watchdog if the time happens to expire. The 'watchdog' property is never addressed in the
_LoggedRobot_ class.

# TimedRobot

Before we dive into the [LoggedTimedCommandRobot](#loggedtimedcommandrobot), we will take a look at the
[TimedRobot]https://robotpy.readthedocs.io/projects/wpilib/en/latest/wpilib/TimedRobot.html) and the
[TimedCommandRobot](#timedcommandrobot) classes that we are wanting to also support. We need to understand what it
does or doesn't do with the _IterativeRobotBase_ and any changes to the 'loopFunc' functionality

## \__init__

Just a default period of 20ms. No new attributes introduced.

Defines the default as 'kDefaultPeriod' and is in milliseconds instead of _default_period_ that **pykit** defines
which is in seconds. _TimedRobot_ does not seem to use this value anywhere, but the derived _TimedCommandRobot_
that we also want to use also uses this as the default optional value for the period and divides that by 1000
in it's \__init__ function.

In the C++ constructor, a priority queue of 'callback' functions are kept and these are serviced during
the 'startCompetion' 'forever' loop. Callbacks are added via the C++ _AddPeriodic_ class method call.
The adds the 'LoopFunc()' as the first one, so this is where the 'thread' safe comment from [addPeriodic](#addperiodic)
comes from.

Also in the c++ class, a notifier is created, so we do not need the hal.Notifier() capability from the _logged_robot_
class.

## startCompetition

In the C++ implementation that this class wraps, the _startCompetition_ function does all the robot work just
like **pykit**'s pure-python function did. Here are the actual steps that it does since the _LoggedTimedCommandRobot_
that we are trying to create will have to use this C++ function for its main loop.

```text

    Step - 1
    call RobotInit()            # Just like 'LoggedRobot'

    if simulation:
        call SimulationInit()   # Also just like 'LoggedRobot'

 ** Step - 2
    call hal.observeUserProgramStarting  # This is the first divergence from 'LoggedRobot'. In 'LoggedRobot'
                                         # a call to initialize 'initEnd' and Logger.periodicAfterUser() is
                                         # made. Then after 'LoggedRobot' calls this, it calls the
                                         # 'Logger.StartReceiver()' function.
    Step - 3
    
    Now Start the big 'forever' loop for this class
```

So the first major sequence between this class and **pykit**'s class function has occurred.

The steps taken in the _TimedRobot_ 'forever' loop are a bit more complicated than **pykit**'s due to its
use of the priority based callback list. The callback priority mechanism is based on a vector pair where the
first is the robot's period and the second is an offset. Both units are in microseconds.

The 'LoopFunc()' callback has a vector of <period>, so any other callbacks (which will be the command scheduler
once we get to the _TimedCommandRobot_ will likely be later.

```text
while True:

    # Step - 3a
    pop the first callback (LoopFunc()) off the callback list
    
    calculate the expiration time and setup the NotifierAlarm to occur at the proper count
    perform the wait.
                        # Up to this point, this is pretty much the same as Step - 5a in the LoggerRobot
                        # class
    # Step - 3b
    Set 'm_loopStartTimeuS to the FPGA time

    # Step - 3c
    call the first callback (LoopFunc())    # Run all those steps in LoopFunc section previously discussed

    # Step - 3d
    push the callback back onto the queue (so it is available again on the next pass)

    # Step - 3e
    iterate over the remaining (if any callbacks)
```

So if we were just to implement our new class now, all we need to do is get the oldest callback and
then create another callback that does the 'Logger.periodicAfterUser' call. But we do not have access
in python to that list, so knowledge of what is getting pushed is needed.

## endCompetition

The C++ class stops the notifier.

## addPeriodic

This is an new function (empty) that is introduced and is a callback that is ran at a specific period with
a stating time offset. This is scheduled on TimedRobot's Notifier, so TimedRobot and the callback run
synchronously. Interactions between them are thread-safe.

The LoopFunc() that we care so much about timing is added as the first callback. No others are in this class, but
when we get to the 'TimedCommandRobot', it will add another one to run the command scheduler.

## getLoopStartTime

C++ class returns the 'm_loopStartTimeUs' value. This is the start of the current loop in microseconds and
is equivalent to the _LoggedRobot_'s _periodicBeforeStart_ local value.

# TimedCommandRobot (commands2)

[TimedCommandRobot](https://robotpy.readthedocs.io/projects/commands-v2/en/latest/commands2/TimedCommandRobot.html)
is provided by the **commands2** module and is a pure-python class implementation.

## \__init__

This is the only function and it will call the _TimedRobot_ 'addPeriodic' to schedule the commands to run
after the 'loopFunc'. The default 'offset' is 5 milliseconds. There will not be a delay after the 'loopFunc'
call, the offset just guarantees it does not run first.

# LoggedTimedCommandRobot

Now, what you've all been anxiously waiting for :-) **The _LoggedTimedCommandRobot_ class**..

See the code..

