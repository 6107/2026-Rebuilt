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

from rev import REVLibError

logger = logging.getLogger(__name__)


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
