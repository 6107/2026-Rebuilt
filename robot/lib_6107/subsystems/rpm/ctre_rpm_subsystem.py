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

from phoenix6 import StatusCode, StatusSignal
from phoenix6.configs import TalonFXConfiguration
from phoenix6.controls import VelocityVoltage
from phoenix6.hardware import TalonFX
from phoenix6.signals import NeutralModeValue
from phoenix6.units import ampere, rotation, rotations_per_second, volt
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, radians, radians_per_second, revolutions_per_minute, \
    rotationsPerMinuteToRadiansPerSecond, rotationsToRadians

from constants import DEFAULT_FREQUENCY
from lib_6107.pykit.autolog import autologgable_output
from lib_6107.subsystems.rpm.rpm_subsystem import ControllerType, RpmMechanismIO, RpmSubsystem
from lib_6107.util.phoenix6_signals import Phoenix6Signals
from lib_6107.util.phoenix6_utils import handle_faults, try_until_ok

logger = logging.getLogger(__name__)


class CtreRpmConfig:
    """
    CTRE specific.  Currently equivalent to the base class RpmSubsystem
    """
    proportional_coefficient = 0.0001  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0           # kI - If you haven’t been where you want to be for a while, apply more effort
                                       #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0         # kD - If you’re getting close to where you want to be, slow down.
    izone = None                       # Not used in CTRE
    imax_accum = None                  # Not used in CTRE
    velocity_feedforward = None

    limit_current: amperes = 40

    # Following are optional and do not need to be provided by the
    # derived class unless they need to be overridden
    gear_reduction = 1.0
    measurement_std_dev = [0.0, 0.0]
    max_rpm: Optional[revolutions_per_minute] = None  # Must be set by subclass


#@autologgable_output
class CtreRpmSubsystem(RpmSubsystem):
    """
    A subsystem with a single motor that typically has an RPM goal.  This will provide a
    base-class that has a PID Controller that has velocity (rotational) goal to achieve
    and maintain.
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool, name: str,
                 motor: DCMotor, controller_type: ControllerType, constants: CtreRpmConfig,
                 long_name: Optional[str] = None,
                 coast: Optional[bool] = True,
                 persist_config: Optional[bool] = False) -> None:

        ###########################################
        # Base class it first
        super().__init__(container, can_device_id, inverted, name,
                         controller_type, constants, long_name)

        ###########################################
        # Now CTRE specific
        # Sanity / defaults check
        self._constants: CtreRpmConfig = self._validate_constants()

        # Set up the motor controller
        match controller_type:
            case ControllerType.KrakenX60:
                self._motor: TalonFX = TalonFX(self._device_id, "")

            case _:
                raise NotImplementedError(f"Unsupported controller type: {controller_type}")

        self._velocity_request: VelocityVoltage = VelocityVoltage(0)

        self._applied_output: StatusSignal[volt] = self._motor.get_motor_voltage(False)
        self._velocity: StatusSignal[rotations_per_second] = self._motor.get_velocity(False)
        self._supply_current: StatusSignal[ampere] = self._motor.get_supply_current(False)
        self._position: StatusSignal[rotation] = self._motor.get_position(False)

        status = StatusSignal.set_update_frequency_for_all(DEFAULT_FREQUENCY,
                                                           self._applied_output,
                                                           self._velocity,
                                                           self._supply_current,
                                                           self._position)
        if status != StatusCode.OK:
            status = self._motor.optimize_bus_utilization()

            if status != StatusCode.OK:
                logger.warning(f"{self.getName()}: Error during signal bus optimization: {status}")

        Phoenix6Signals.register_signals(self._applied_output,
                                         self._velocity,
                                         self._supply_current,
                                         self._position)

        ###########################################
        # Finally have base class handle any remaining post_init attributes
        super().post_init(bool(coast), bool(persist_config))

    def post_init(self, coast: bool, persist_config: bool) -> None:
        # Bass class will validate the config
        super().post_init(coast, persist_config)

        # Now apply it
        config = self._motor_config(coast)

        # persist = PersistMode.kPersistParameters if persist_config else PersistMode.kNoPersistParameters
        config_status = try_until_ok(self.getName(), 5, lambda: self._motor.configurator.apply(config))

        # Check if the device was successfully configured and can be reached over the
        # CAN bus.
        self._is_connected = self._check_is_connected(config_status)

        # Note: No encoders at this tim
        #self._encoder: SupportedEncoders = self._motor.getEncoder()

    def _validate_constants(self) -> CtreRpmConfig:
        """
        Validate that the constants passed in have values/properties this class needs. They
        can be None if you want this class to use a default value (often zero), but they do need
        to exist as an explicit attribute of the object passed in
        """
        return super()._validate_constants()        # TODO: Remove if we do not add to this method

    def _motor_config(self, coast: bool) -> TalonFXConfiguration:
        """
        Motor config for the intake Indexer. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        match self._controller_type:
            case ControllerType.KrakenX60:
                config = TalonFXConfiguration()

            case _:
                raise NotImplementedError("CtreRpmSubsystem._motor_config: Unsupported controller type")

        config.current_limits.supply_current_limit = self._constants.limit_current
        config.motor_output.neutral_mode = NeutralModeValue.COAST if coast else NeutralModeValue.BRAKE

        # Closed loop configuration parameters, slot=0
        #
        #   P:  If you’re not where you want to be, get there.
        #
        #   I:     If you haven’t been where you want to be for a while, apply more effort
        #          to get there”, since it really isn’t about speed.
        #
        #   D:     If you’re getting close to where you want to be, slow down.
        #
        #   IZone: If you are really far from where you want to be, don’t start applying
        #          more effort to get there until you are within this margin
        #
        # slot0 = ClosedLoopSlot(ClosedLoopSlot.kSlot0)
        # (
        #     config.closedLoop
        #     # .IMaxAccum(0.03, slot=slot0)
        #     # .IZone(3, slot=slot0)
        #     .pid(p=self._constants.proportional_coefficient,  # Slot 0 for position control
        #          i=self._constants.integral_coefficient,
        #          d=self._constants.derivative_coefficient,
        #          slot=slot0)
        #     .positionWrappingEnabled(True)
        #     .outputRange(-1, 1)
        # )
        config.slot0.k_p = self._constants.proportional_coefficient
        config.slot0.k_i = self._constants.integral_coefficient
        config.slot0.k_d = self._constants.derivative_coefficient

        if self._constants.velocity_feedforward is not None:
            config.slot0.k_v = self._constants.velocity_feedforward

        return config

    def _check_is_connected(self, config_status: StatusCode | None) -> bool:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        return config_status == StatusCode.OK or (config_status is None and
                                                  StatusSignal.is_all_good(self._applied_output,
                                                                           self._velocity,
                                                                           self._supply_current,
                                                                           self._position))

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        match self._controller_type:
            case ControllerType.KrakenX60:
                return self._is_connected
        return False

    @property
    def velocity_in_rps(self) -> radians_per_second:
        rps: radians_per_second = rotationsToRadians(self._velocity.value)
        return -rps if self._inverted else rps

    @property
    def position(self) -> radians:
        return rotationsToRadians(self._position.value)

    def _set_velocity_goal(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None) -> None:
        self._velocity_tolerance = rpm_tolerance or 0.0
        self._velocity_goal, previous = max(0.0, min(self._constants.max_rpm, abs(rpm))), self._velocity_goal

        if self._velocity_goal != previous or self._velocity_tolerance != self.tolerance:
            logger.info(f"{self.getName()}: Setting goal RPM to {self._velocity_goal}. previous: {previous}")
            logger.info(
                f"{self.getName()}: current PID controller setpoint before command: {self._pid_controller.getSetpoint()}")

            # Set velocity goal. Convert RPM to rps
            rps: radians_per_second = rotationsPerMinuteToRadiansPerSecond(self._velocity_goal)
            self._velocity_request.with_velocity(rps)

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        inputs.mechanism_connected = self.is_connected

        # TODO: Is position really important (maybe for playback?)
        inputs.mechanism_position = self.position
        inputs.mechanism_velocity = self.velocity_in_rps
        inputs.mechanism_applied_voltage = self._applied_output.value
        inputs.mechanism_supply_current = self._supply_current.value

    def fault_detection(self, state: str, clear: Optional[bool] = True, notify: Optional[bool] = True) -> None:
        """
        This routine is responsible for reading any existing faults and based
        input parameters, report them for display, and possibly clear them

        All faults detected always results in a warning log message, so please be
        aware of this if you do not clear them

        TODO: Good thing for a base class, don't you think
        """
        # For Rev Robotics, the faults are a bitmask
        handle_faults(self.getName(), state, self._motor, clear=clear, notify=notify)
