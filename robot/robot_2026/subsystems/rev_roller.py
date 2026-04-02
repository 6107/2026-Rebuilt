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
from typing import Optional

from wpimath.system.plant import DCMotor
from wpimath.units import amperes, revolutions_per_minute

from lib_6107.pykit.autolog import autologgable_output
from lib_6107.subsystems.rpm_subsystem import ControllerType, RpmConfig, RpmSubsystem

logger = logging.getLogger(__name__)


class IntakeConstants(RpmConfig):
    proportional_coefficient = 10  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0  # kI - If you haven’t been where you want to be for a while, apply more effort
    #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0  # kD - If you’re getting close to where you want to be, slow down.
    izone = None  # If you are really far from where you want to be, don’t start applying
    #      more effort to get there until you are within this margin

    limit_current: amperes = 40


@autologgable_output
class RevIntakeRoller(RpmSubsystem):
    """
    Intake Roller Motor.

    This is the rolly-grabber at the front of the intake.
    """

    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "Roller",
                         DCMotor.NEO(1), ControllerType.SparkMax, IntakeConstants(),
                         long_name="Intake/Roller",
                         coast=True,
                         persist_config=persist_config)
        self._initialized = True

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self._container.intake_pivot.enabled and self.is_initialized # Use a single enable for all the intake frontend

    def intake_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Consume fuel from the playing area
        logger.info(f"Indexer: Intake Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(rpm, rpm_tolerance)

    def expel_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Expel any fuel we may have
        logger.info(f"Roller: Expel Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(-rpm, rpm_tolerance)  # TODO: Should this be separate function or maybe lower value
