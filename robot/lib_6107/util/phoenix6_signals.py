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

from enum import Enum
from typing import Set

logger = logging.getLogger(__name__)

try:
    from phoenix6 import StatusCode, StatusSignal

except ImportError:
    class StatusCode(Enum):
        OK = 0

    class StatusSignal:
        name = "Not Supported"
        @staticmethod
        def refesh_all() -> StatusCode:
            return StatusCode.OK


class Phoenix6Signals:
    """
    Optimizer for CTRE devices

    """
    _signals: Set[StatusSignal] = set()

    @classmethod
    def register_signal(cls, signal: StatusSignal) -> None:
        if signal not in cls._signals:
            cls._signals.add(signal)
        else:
            logging.warning(f"Signal {signal.name} already registered")

    @classmethod
    def register_signals(cls, *signals: StatusSignal) -> None:
        for signal in signals:
            cls.register_signal(signal)

    @classmethod
    def refresh(cls) -> StatusCode:
        """
        In robot.robotPeriodic, call this to request all status to be updated at once. Documentation
        says it is faster
        """
        return StatusSignal.refresh_all(*list(cls._signals))
