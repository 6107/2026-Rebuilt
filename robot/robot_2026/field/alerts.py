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

from pykit.alertlogger import AlertLogger
from wpilib import Alert, DriverStation, RobotBase

from robot_2026.util.preflight import PreflightChecklist


class RobotAlerts:
    def __init__(self, container: 'RobotContainer'):
        self._container = container

        # TODO: Need to validate all alerts so we can trust them
        AlertLogger.registerGroup("Alerts")

        self.driver_disconnected = Alert("Driver controller disconnected (port 0)",
                                         Alert.AlertType.kWarning)
        self.operator_disconnected = Alert("Operator controller disconnected (port 1)",
                                           Alert.AlertType.kWarning)
        self.dead_in_the_water_alert = Alert("No auto selected!!!",
                                             Alert.AlertType.kWarning)
        self.flywheel_disabled_alert = Alert("Flywheel is disabled!!!",
                                             Alert.AlertType.kWarning)
        self.climber_disabled_alert = Alert("Climber is disabled!!!",
                                            Alert.AlertType.kWarning)
        self.intake_disabled_alert = Alert("Intake is disabled!!!",
                                           Alert.AlertType.kWarning)

        self.shift_active_alert = Alert("SHIFT ACTIVE!", Alert.AlertType.kInfo)  # TODO: also maybe a vibrate?
        self.shift_active_alert.set(True)

        self.usbAlert = Alert("No USB Drive in robot!", Alert.AlertType.kError)

        if RobotBase.isReal() and not os.path.exists("/U/logs"):
            self.usbAlert.set(True)

        self._preflight_alert = Alert("preflight checking not complete",
                                      Alert.AlertType.kError)
        # preflight checklist
        AlertLogger.registerGroup("preflight")
        self._preflight = PreflightChecklist()

    def update(self) -> None:
        self.driver_disconnected.set(not DriverStation.isJoystickConnected(0))
        self.operator_disconnected.set(not DriverStation.isJoystickConnected(1))

        self.dead_in_the_water_alert.set(self._container.auto_chooser.getSelected() == self._container.get_do_nothing)
        self.flywheel_disabled_alert.set(not self._container.flywheel.enabled)
        self.climber_disabled_alert.set(not self._container.climber.enabled)
        self.intake_disabled_alert.set(not self._container.intake_pivot.enabled)  # Controls entire system

        self._preflight_alert.set(not self._preflight.is_complete())

    def preflight_update(self) -> None:
        self._preflight.update()
