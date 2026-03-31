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
from commands2.button import Trigger
from lib_6107.subsystems.rpm_subsystem import ControllerType, RpmConfig, RpmSubsystem
from lib_6107.util.competition import event_active
from pykit.autolog import autologgable_output
from pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from typing import Optional
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, revolutions_per_minute

logger = logging.getLogger(__name__)

# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class FlywheelConstants(RpmConfig):
    proportional_coefficient = 10  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0  # kI - If you haven’t been where you want to be for a while, apply more effort
    #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0  # kD - If you’re getting close to where you want to be, slow down.
    izone = None  # If you are really far from where you want to be, don’t start applying
    #      more effort to get there until you are within this margin

    limit_current: amperes = 40


@autologgable_output
class RevFlywheel(RpmSubsystem):
    """
    Rev NEO 21-1650
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "Flywheel",
                         DCMotor.NEO(1), ControllerType.SparkMax, FlywheelConstants(),
                         long_name="Intake/Flywheel",
                         coast=True,
                         persist_config=persist_config)

        self._enable_chooser = LoggedNetworkBoolean("Flywheel/Enabled",
                                                    defaultValue=event_active())
        self._initialized = True

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self.is_initialized and self._enable_chooser.value

    def shoot_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Shoot fuel toward whatever we are aimed at.
        logger.info(f"Roller: Expel Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(rpm, rpm_tolerance)
