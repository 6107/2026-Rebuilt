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
# From 1757-Westwood Robotics: https://github.com/1757WestwoodRobotics/2026-Rebuilt

from pykit.logger import Logger, RobotController


class LogTracer:
    """
    LogTracer provides a wrapper around pykit loggings where various blocks (prefix)
    of code can be correlated. By resetting the outer prefix, you can begin a new
    'related' block of log messages and watch progress through that block (with the
    'record' method). At the end, call 'recordToTotal()' to record a total time
    through that block of code.
    """
    _inner_start: float = 0.0
    _outer_start: float = 0.0

    _prefix: str = ""

    @classmethod
    def resetOuter(cls, prefix: str) -> None:
        """
        Start a new block of code that you want to time.
        """
        cls._outer_start = RobotController.getFPGATime()
        cls.reset()
        cls._prefix = prefix

    @classmethod
    def reset(cls) -> None:
        """
        Clear current in start time for this block of logging / code
        """
        cls._inner_start = RobotController.getFPGATime()

    @classmethod
    def record(cls, action: str) -> None:
        """
        Record a location within the block of code of what just finished (the action).
        """
        now = RobotController.getFPGATime()
        Logger.recordOutput(f"LogTracer/{cls._prefix}/{action}MS", (now - cls._inner_start) / 1000.0)
        cls._inner_start = now

    @classmethod
    def recordTotal(cls) -> None:
        """
        Typically called at the end of a block of logging, record the time it took to get to
        this point.
        """
        now = RobotController.getFPGATime()
        Logger.recordOutput(f"LogTracer/{cls._prefix}/TotalMS", (now - cls._outer_start) / 1000.0)
