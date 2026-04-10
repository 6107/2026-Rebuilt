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
from dataclasses import dataclass
from typing import Optional

from wpimath.geometry import Pose2d
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, degrees, meters, revolutions_per_minute

from lib_6107.pykit.autolog import autolog
from lib_6107.pykit.logger import Logger
from lib_6107.pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from lib_6107.subsystems.rpm.rev_rpm_subsystem import RevRpmConfig, RevRpmSubsystem
from lib_6107.subsystems.rpm.rpm_subsystem import ControllerType, RpmMechanismIO
from lib_6107.util.competition import event_active
from robot_2026.field.field_2026 import RebuiltField as Field
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)

# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class FlywheelConstants(RevRpmConfig):
    # Configure PID coefficients (values will vary by mechanism)
    # kF is often calculated as 1 / (Max Free Speed)
    # Example for NEO (~5676 RPM): 1 / 5676 = 0.000176

    proportional_coefficient = 0.0020645  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0              # kI - If you haven’t been where you want to be for a while, apply more effort
                                          #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0            # kD - If you’re getting close to where you want to be, slow down.
    izone = None                          #      If you are really far from where you want to be, don’t start applying
                                          #      more effort to get there until you are within this margin

    max_rpm: revolutions_per_minute = 5676.0  # Neo
    limit_current: amperes = 40

    velocity_feedforward = 1.0/5676.0

# TODO: Following are estimates. Need to verify
# At +/- the tolerance, there is an 80% chance fuel shot will score in the hub
FLYWHEEL_MAX_RANGE: meters = 7.0  # TODO: validate
FLYWHEEL_MIN_RANGE: meters = 1.0  # TODO: validate
FLYWHEEL_ANGLE_TOLERANCE: degrees = 4.0  # TODO: validate


class FlywheelIO:
    """
    Drive I/O for a mechanism that has an RPM Goal
    """

    @autolog
    @dataclass
    class FlywheelIOIOInputs(RpmMechanismIO.RpmMechanismIOInputs):
        # mechanism_connected: bool = False
        #
        # mechanism_position: radians = 0.0
        # mechanism_velocity: radians_per_second = 0.0      TODO: Keep these until we know we can subclass this way
        # mechanism_applied_voltage: volts = 0.0
        # mechanism_supply_current: amperes = 0.0
        ######################
        # Flywheel specific
        hub_in_range: bool = False
        aimed_at_hub: bool = False
        rpm_at_goal: bool = False

    def __init__(self, name: str) -> None:
        self.name = name

    def updateInputs(self, inputs: FlywheelIOIOInputs) -> None:
        """
        Update the Flywheel I/O inputs.

        Args:
            inputs (FlywheelIOIOInputs): The drive I/O inputs to update.
        """
        pass


#@autologgable_output
class RevFlywheel(RevRpmSubsystem):
    """
    Rev NEO 21-1650
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int,
                 inverted: bool, persist_config: Optional[bool] = False) -> None:
        super().__init__(container, can_device_id, inverted, "Flywheel",
                         DCMotor.NEO(1), ControllerType.SparkMax, FlywheelConstants(),
                         long_name="Intake/Flywheel",
                         persist_config=persist_config)

        # NOTE: We take ownership to the pykit IO Inputs from base class here!
        self._inputs = FlywheelIO.FlywheelIOIOInputs()
        self._field: Field = container.field

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

    @property
    def hub_in_range(self) -> bool:
        return self._inputs.hub_in_range

    @property
    def aimed_at_hub(self) -> bool:
        return False    # self._inputs.aimed_at_hub

    @property
    def rpm_at_goal(self) -> bool:
        return False  # self._inputs.rpm_at_goal

    def updateInputs(self, inputs: FlywheelIO.FlywheelIOIOInputs) -> None:
        # Handles base class IO inputs directly here
        inputs.mechanism_connected = self.is_connected

        inputs.mechanism_position = self.position
        inputs.mechanism_velocity = self.velocity_in_rps
        inputs.mechanism_applied_voltage = self._motor.getAppliedOutput()
        inputs.mechanism_supply_current = self._motor.getOutputCurrent()

        # Flywheel specific below
        pose: Pose2d = self._container.field2d.getRobotPose()

        if self._field.in_my_alliance_zone(pose.x):
            distance: meters | None = self._field.distance_to_hub

            inputs.hub_in_range = distance is not None and FLYWHEEL_MIN_RANGE <= distance <= FLYWHEEL_MAX_RANGE

            # TODO: Should angle tolerance be relative to ?
            angle: degrees = None  # TODO extract           Make angle relative to center of hub
            inputs.aimed_at_hub = angle is not None and abs(angle) <= FLYWHEEL_ANGLE_TOLERANCE

            rpm = self.velocity_in_rpm
            tolerance = self._velocity_tolerance
            inputs.rpm_at_goal = rpm - tolerance <= self.goal <= rpm + tolerance
        else:
            inputs.hub_in_range = False
            inputs.aimed_at_hub = False
            inputs.rpm_at_goal = False

    def periodic(self) -> None:
        if self.is_initialized:
            LogTracer.resetOuter(f"{self.getName()} periodic")

            self.updateInputs(self._inputs)

            Logger.processInputs(self.getName(), self._inputs)
            LogTracer.record("UpdateInputs")

            # TODO: Look what we need to do if we provide replay?
            Logger.recordOutput(f"{self._long_name}/goal", self.goal)
            Logger.recordOutput(f"{self._long_name}/current", self.velocity_in_rpm)
            Logger.recordOutput(f"{self._long_name}/tolerance", self.tolerance)

            # Flywheel specific here
            Logger.recordOutput("Flywheel/HubInRange", self.hub_in_range)
            Logger.recordOutput("Flywheel/AimedAtHub", self.aimed_at_hub)
            Logger.recordOutput("Flywheel/RpmAtGoal", self.rpm_at_goal)

            LogTracer.recordTotal()

            # Update SmartDashboard for this subsystem at a rate slower than the period
            counter = self._robot.counter
            if counter % 100 == 0 or (self._robot.counter % 13 == 0 and
                                      self._robot.isEnabled()):
                self.dashboard_periodic()

    def dashboard_periodic(self) -> None:
        super().dashboard_periodic()
        # TODO: Following are debug visualization. May remove later

        # SmartDashboard.putNumber(f"{self._long_name}/Goal", self.goal)
        # SmartDashboard.putNumber(f"{self._long_name}/Current", self.velocity_in_rpm)
        # SmartDashboard.putNumber(f"{self._long_name}/Tolerance", self.tolerance)
        # SmartDashboard.putNumber(f"{self._long_name}/Voltage", self._motor.getAppliedOutput())
        # SmartDashboard.putNumber(f"{self._long_name}/Current", self._motor.getOutputCurrent())

