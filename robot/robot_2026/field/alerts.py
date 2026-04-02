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

from wpilib import Alert, DriverStation, RobotBase

from lib_6107.pykit.alertlogger import AlertLogger
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
        if container.flywheel is not None:
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

        def missing_or_disabled(device) -> bool:
            return device is None or not device.enabled

        self.dead_in_the_water_alert.set(self._container.auto_chooser.getSelected() == self._container.get_do_nothing)

        if self._container.flywheel is not None:
            self.flywheel_disabled_alert.set(missing_or_disabled(self._container.flywheel))
        self.climber_disabled_alert.set(missing_or_disabled(self._container.climber))
        self.intake_disabled_alert.set(missing_or_disabled(self._container.intake_pivot))  # Controls entire system

        self._preflight_alert.set(not self._preflight.is_complete())

    def preflight_update(self) -> None:
        self._preflight.update()
