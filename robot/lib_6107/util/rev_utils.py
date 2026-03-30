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
from typing import Callable, Optional

from rev import REVLibError, SparkBase, SparkFlex, SparkFlexSim, SparkMax, SparkMaxSim

from lib_6107.util.elastic_utils import Notification, NotificationLevel, send_notification

logger = logging.getLogger(__name__)

SparkDevices = SparkFlex | SparkMax | SparkFlexSim | SparkMaxSim

REV_WARNINGS = ("brownout", "escEeprom", "extEeprom", "hasReset", "overcurrent", "sensor", "stall", "other")
REV_FAULTS = ("can", "escEeprom", "firmware", "gateDrover", "motorType", "sensor", "temperature", "other")

def try_until_ok(what: str, attempts: int, command: Callable[[], REVLibError]) -> REVLibError:
    """
    Repeat a command for certain number of attempts or until it succeeds
    """
    assert attempts > 0, f"{what} -> {str(command)}: Attempts must be greater than 0"
    prev_code: Optional[REVLibError] = None

    for attempt in range(attempts):
        code: REVLibError = command()

        if code == REVLibError.kOk:
            if attempt > 0:
                logger.warning(
                    f"{what}: {str(command)} succeeded on {attempt} attempt last failed status: {prev_code.value} - {prev_code.name}")
            return code
        prev_code = code

    logger.error(f"{what}: {str(command)} failed after {attempts} attempts. Final Status: {code.value} - {code.name}")
    return code


def handle_faults(name: str, state: str, device: SparkDevices, clear: Optional[bool] = True,
                  notify: Optional[bool] = True) -> None:
    """
    This routine is responsible for reading any existing faults and based
    input parameters, report them for display, and possibly clear them

    All faults detected always results in a warning log message, so please be
    aware of this if you do not clear them

    TODO: Good thing for a base class, don't you think
    """

    # For Rev Robotics, the faults are a bitmask
    def elastic_notification(title: str, message: str, level: NotificationLevel) -> None:
        send_notification(Notification(level=level,
                                       title=title,
                                       description=message))

    try:
        warnings = device.getWarnings()
        sticky_warnings = device.getStickyWarnings()
        sticky_issues_found = False

        if warnings.rawBits != 0:
            active = {issue for issue in REV_WARNINGS
                      if hasattr(warnings, issue) and getattr(warnings, issue, False)}

            msg = f"Device {name} during {state}: ({", ".join(active)})"
            if notify:
                elastic_notification("Active Warning", msg, NotificationLevel.WARNING)
            logger.warning(f"Active Warning: {msg}")

        if sticky_warnings.rawBits != 0:
            sticky_issues_found = True
            sticky = {issue for issue in REV_WARNINGS
                      if hasattr(sticky_warnings, issue) and getattr(sticky_warnings, issue, False)}

            msg = f"Device {name} during {state}: ({", ".join(sticky)})"
            if notify:
                elastic_notification("'Sticky' Warning", msg, NotificationLevel.WARNING)
            logger.warning(f"'Sticky' Warning: {msg}")

        faults: SparkBase.Faults = device.getFaults()
        sticky_faults = device.getStickyFaults()

        if faults.rawBits != 0:
            active = {issue for issue in REV_FAULTS
                      if hasattr(faults, issue) and getattr(faults, issue, False)}

            msg = f"Device {name} during {state}: ({", ".join(active)})"
            if notify:
                elastic_notification("Active Fault", msg, NotificationLevel.WARNING)
            logger.warning(f"Active Fault: {msg}")

        if sticky_faults.rawBits != 0:
            sticky_issues_found = True
            sticky = {issue for issue in REV_FAULTS
                      if hasattr(sticky_faults, issue) and getattr(sticky_faults, issue, False)}

            msg = f"Device {name} during {state}: ({", ".join(sticky)})"
            if notify:
                elastic_notification("'Sticky' Fault", msg, NotificationLevel.WARNING)
            logger.warning(f"'Sticky' Fault: {msg}")

        # Clear them?
        if clear and sticky_issues_found:
            device.clearFaults()

    except Exception as e:
        logger.exception(f"REV:handle_faults: Exception while processing faults: {e}")
