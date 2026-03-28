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
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from wpilib import SendableChooser, SmartDashboard
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, revolutions_per_minute

from lib_6107.subsystems.rpm_subsystem import ControllerType, RpmSubsystem

logger = logging.getLogger(__name__)

# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class ShooterConstants:
    PROPORTIONAL_COEFFICIENT = 0.5 / 10000  # kP        # TODO: All of this needs tuning
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0.0 / 10000

    VELOCITY_FEEDFORWARD = None  # 18.5 / 10000
    IMAX_ACCUM = None  # 0.03
    IZONE = None  # 3

    LIMIT_CURRENT: amperes = 40

    GEAR_REDUCTION = 6.75  # TODO: Get number
    MEASUREMENT_STD_DEV = [0.0, 0.0]  # TODO: Get number for noise
    MAX_RPM: revolutions_per_minute = 5676.0


@autologgable_output
class RevShooter(RpmSubsystem):
    """
    Rev NEO 21-1650
    """

    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "roller",
                         DCMotor.NEO(1), ControllerType.SparkMax, ShooterConstants(),
                         long_name="Intake/Roller",
                         coast=True,
                         persist_config=persist_config)

        # TODO: Remove following once all works
        # self._enable_chooser = LoggedDashboardChooser("Shooter Enabled")
        self._enable_chooser = LoggedDashboardChooser("Shooter Enabled")

        self._enable_chooser.setDefaultOption("False", False)
        self._enable_chooser.addOption("True", True)

        if isinstance(self._enable_chooser, SendableChooser):
            SmartDashboard.putData("Shooter Enabled", self._enable_chooser)
        elif isinstance(self._enable_chooser, LoggedDashboardChooser):
            pass

        self._initialized = False

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self._enable_chooser.getSelected() is True and self.is_initialized

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    def shoot_fuel(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None):
        # Shoot fuel toward whatever we are aimed at.
        logger.info(f"Roller: Expel Fuel, currently (RPM): {self.velocity_in_rpm}, Goal: {self.goal}")

        self._set_velocity_goal(rpm, rpm_tolerance)
