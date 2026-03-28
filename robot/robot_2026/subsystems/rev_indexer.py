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

from commands2.button import Trigger
from pykit.autolog import autologgable_output
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, revolutions_per_minute

from lib_6107.subsystems.rpm_subsystem import ControllerType, RpmSubsystem

logger = logging.getLogger(__name__)


class IntakeConstants:
    PROPORTIONAL_COEFFICIENT = 10  # kP
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0  # kD

    VELOCITY_FEEDFORWARD = None
    IMAX_ACCUM = None
    IZONE = None

    LIMIT_CURRENT: amperes = 30

    GEAR_REDUCTION = 6.75  # TODO: Get number
    MEASUREMENT_STD_DEV = [0.0, 0.0]  # TODO: Get number for noise
    MAX_RPM: revolutions_per_minute = 5676.0

    TARGET_RPM: revolutions_per_minute = 100  # Start slow


@autologgable_output
class RevIntakeIndexer(RpmSubsystem):
    """
    Intake Indexer Motor.

    This is the rolly-grabber at the front of the intake.
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "Indexer",
                         DCMotor.NEO(1), ControllerType.SparkMax, IntakeConstants(),
                         long_name="Intake/Indexer",
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
        return self._container.intake_pivot.enabled  # Use a single enable for all the intake frontend

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled and self.is_initialized)

    def intake_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Consume fuel from the playing area
        logger.info(f"Indexer: Intake Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(rpm, rpm_tolerance)

    def expel_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Expel any fuel we may have
        logger.info(f"Indexer: Expel Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(-rpm, rpm_tolerance)  # TODO: Should this be separate function or maybe lower value
