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

import os

from wpilib import Alert
from lib_6107.util.alerts import RobotAlerts

class MyRobotAlerts(RobotAlerts):
    def __init__(self, container: "RobotContainer"):
        super().__init__(container)

        if container.flywheel is not None:
            self.flywheel_disabled_alert = Alert("Flywheel is disabled!!!",
                                                 Alert.AlertType.kWarning)
        self.climber_disabled_alert = Alert("Climber is disabled!!!",
                                            Alert.AlertType.kWarning)
        self.intake_disabled_alert = Alert("Intake is disabled!!!",
                                           Alert.AlertType.kWarning)

        self.shift_active_alert = Alert("SHIFT ACTIVE!", Alert.AlertType.kInfo)  # TODO: also maybe a vibrate?
        self.shift_active_alert.set(True)


    def update(self) -> None:
        super().update()

        def missing_or_disabled(device) -> bool:
            return device is None or not device.enabled

        if self._container.flywheel is not None:
            self.flywheel_disabled_alert.set(missing_or_disabled(self._container.flywheel))

        self.climber_disabled_alert.set(missing_or_disabled(self._container.climber))
        self.intake_disabled_alert.set(missing_or_disabled(self._container.intake_pivot))  # Controls entire system

        self._preflight_alert.set(not self._preflight.is_complete())

