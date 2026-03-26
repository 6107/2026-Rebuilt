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

from lib_6107.subsystems.rpm_subsystem import MotorType, RpmSubsystem

logger = logging.getLogger(__name__)


class IntakeConstants:
    PROPORTIONAL_COEFFICIENT = 10  # kP
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0  # kD
    LIMIT_CURRENT: amperes = 30

    GEAR_REDUCTION = 6.75  # TODO: Get number
    MEASUREMENT_STD_DEV = [0.0, 0.0]  # TODO: Get number for noise
    MAX_RPM = 6784
    TARGET_RPM: revolutions_per_minute = 100  # Start slow


@autologgable_output
class RevIntakeRoller(RpmSubsystem):
    """
    Intake Roller Motor.

    This is the rolly-grabber at the front of the intake.
    """

    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "roller",
                         DCMotor.NEO(1), MotorType.SparkFlex, IntakeConstants(),
                         long_name="Intake/Roller",
                         coast=True,
                         persist_config=persist_config)

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
        return Trigger(lambda: self.enabled)

    def intake_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Consume fuel from the playing area
        self._set_velocity_goal(rpm, rpm_tolerance)

    def expel_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Expel any fuel we may have
        self._set_velocity_goal(-rpm, rpm_tolerance)  # TODO: Should this be separate function or maybe lower value
