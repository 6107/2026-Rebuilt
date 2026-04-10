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
# Code originally from Westwood-2026-Rebuilt

from wpilib import Alert

from lib_6107.pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean


class PreflightChecklist:
    class PreflightCheck:
        def __init__(self, name: str, key: str, expected: bool):
            self._key = key
            self._name = name
            self._value = LoggedNetworkBoolean(key, not expected)
            self._expected = expected
            self.alert = Alert("preflight", f"Check Failed: {name}", Alert.AlertType.kError)

        def __str__(self) -> str:
            return repr(self)

        def __repr__(self):
            return f"PreflightCheck({self._name}: {self._value.value})"

        def update(self):
            """
            Sets the proper alert for this check based on the value, which should be set by the
            user to indicate whether the check has been completed or not
            """
            self.alert.set(not self.value())

        def value(self) -> bool:
            """
            Returns the value of this check, which should be set by the user to indicate whether
            the check has been completed or not
            """
            return self._value.value == self._expected

        @property
        def name(self) -> str:
            return self._name

    def __init__(self):
        self.checks: list[PreflightChecklist.PreflightCheck] = [
            PreflightChecklist.PreflightCheck(
                "DriverStation restarted?", "Preflight/DSR", True
            ),
            PreflightChecklist.PreflightCheck(
                "Climber Retracted to 8 at last power-off", "Preflight/ClimbZero", True
            ),
            PreflightChecklist.PreflightCheck(
                "Robot has Power", "Preflight/RobotPower", True
            ),
            PreflightChecklist.PreflightCheck(
                "Robot was powered on in starting config", "Preflight/RobotStart", True
            ),
            PreflightChecklist.PreflightCheck(
                "Autonomous is Selected", "Preflight/Autonomous", True
            ),
            # PreflightChecklist.PreflightCheck(
            #     "Autonomous Ending is Selected", "Preflight/AutoEnds", True
            # ),
            PreflightChecklist.PreflightCheck(
                "Robot in correct autonomous position", "Preflight/AutoLoc", True
            ),
            PreflightChecklist.PreflightCheck(
                "Vision returns Results", "Preflight/Vision", True
            ),
        ]

    def update(self):
        for check in reversed(self.checks):
            check.update()

    def is_complete(self) -> bool:
        return all(check.value() for check in self.checks)

    def missing(self) -> list[str]:
        return [check.name for check in self.checks if not check.value()]
