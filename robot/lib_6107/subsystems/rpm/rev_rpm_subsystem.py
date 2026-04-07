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
import math
from typing import Any, Callable, Optional

from rev import ClosedLoopSlot, PersistMode, ResetMode, REVLibError, SparkBase, SparkBaseConfig, SparkFlex, \
    SparkFlexConfig, SparkFlexSim, SparkMax, SparkMaxConfig, SparkMaxSim, SparkRelativeEncoder
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, radians, radians_per_second, revolutions_per_minute

from lib_6107.pykit.autolog import autologgable_output
from lib_6107.subsystems.rpm.rpm_subsystem import ControllerType, RpmConfig, RpmMechanismIO, RpmSubsystem, \
    SupportedClosedLoopControllers, SupportedEncoders
from lib_6107.util.rev_utils import handle_faults, try_until_ok

logger = logging.getLogger(__name__)


class RevRpmConfig(RpmConfig):
    """
    Rev Robotics specific.  Currently equivalent to the base class RpmSubsystem
    """
    proportional_coefficient = 0.0001  # kP - If you’re not where you want to be, get there.
    integral_coefficient = 0           # kI - If you haven’t been where you want to be for a while, apply more effort
                                       #      to get there”, since it really isn’t about speed.
    derivative_coefficient = 0         # kD - If you’re getting close to where you want to be, slow down.
    izone = None                       #      If you are really far from where you want to be, don’t start applying
                                       #      more effort to get there until you are within this margin
    velocity_feedforward = None
    imax_accum = None

    limit_current: amperes = 40

    # Following are optional and do not need to be provided by the
    # derived class unless they need to be overridden
    gear_reduction = 1.0
    measurement_std_dev = [0.0, 0.0]
    max_rpm: Optional[revolutions_per_minute] = None  # Must be set by subclass

#@autologgable_output
class RevRpmSubsystem(RpmSubsystem):
    """
    A subsystem with a single motor that typically has an RPM goal.  This will provide a
    base-class that has a PID Controller that has velocity (rotational) goal to achieve
    and maintain.
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool, name: str,
                 motor: DCMotor, controller_type: ControllerType, constants: RevRpmConfig,
                 long_name: Optional[str] = None,
                 coast: Optional[bool] = True,
                 persist_config: Optional[bool] = False) -> None:

        ###########################################
        # Base class it first
        super().__init__(container, can_device_id, inverted, name,
                         controller_type, constants, long_name)

        ###########################################
        # Now Rev Robotics specific
        # Sanity / defaults check
        self._constants: RevRpmConfig = self._validate_constants()
        self._encoder: SupportedEncoders | None = None

        # Set up the motor controller
        match controller_type:
            case ControllerType.SparkMax:
                self._motor: SparkMax = SparkMax(self._device_id, SparkMax.MotorType.kBrushless)
                if self._is_simulation:
                    self._sim_motor = SparkMaxSim(self._motor, motor)

            case ControllerType.SparkFlex:
                self._motor: SparkFlex = SparkFlex(self._device_id, SparkFlex.MotorType.kBrushless)
                if self._is_simulation:
                    self._sim_motor = SparkFlexSim(self._motor, motor)

            case _:
                raise NotImplementedError(f"Unsupported controller type: {controller_type}")

        self._encoder: SparkRelativeEncoder = self._motor.getEncoder()

        ###########################################
        # Finally have base class handle any remaining post_init attributes
        super().post_init(bool(coast), bool(persist_config))

    def post_init(self, coast: bool, persist_config: bool) -> None:
        # Bass class will validate the config
        super().post_init(coast, persist_config)

        # Now apply it
        persist = PersistMode.kPersistParameters if persist_config else PersistMode.kNoPersistParameters

        config_status = try_until_ok(self.getName(), 5,
                                     lambda: self._motor.configure(self._motor_config(coast),
                                                                   ResetMode.kResetSafeParameters,
                                                                   persist))

        # Check if the device was successfully configured and can be reached over the
        # CAN bus.
        self._is_connected = self._check_is_connected(config_status)

        # Set up the encoders
        self._encoder: SupportedEncoders = self._motor.getEncoder()

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._pid_controller: SupportedClosedLoopControllers = self._motor.getClosedLoopController()
        self._pid_controller.setSetpoint(0.0, SparkBase.ControlType.kVoltage, ClosedLoopSlot(0))

    def _validate_constants(self) -> RevRpmConfig:
        """
        Validate that the constants passed in have values/properties this class needs. They
        can be None if you want this class to use a default value (often zero), but they do need
        to exist as an explicit attribute of the object passed in
        """
        return super()._validate_constants()        # TODO: Remove if we do not add to this method

    def try_until_ok(self, what: str, attempts: int, command: Callable[[], Any]) -> REVLibError:
        return try_until_ok(what, attempts, command)

    def _motor_config(self, coast: bool) -> SparkBaseConfig:
        """
        Motor config for the intake Indexer. Using the default Primary Encoder
        as the Feedback Sensor.
        """
        match self._controller_type:
            case ControllerType.SparkMax:
                config = SparkMaxConfig()

            case ControllerType.SparkFlex:
                config = SparkFlexConfig()

            case _:
                raise NotImplementedError("RevRpmSubsystem._motor_config: Unsupported controller type")

        config = (config
                  .inverted(self._inverted)
                  .smartCurrentLimit(self._constants.limit_current)
                  .setIdleMode(SparkFlexConfig.IdleMode.kCoast if coast else SparkFlexConfig.IdleMode.kBrake)
                  )

        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

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
        slot0 = ClosedLoopSlot(ClosedLoopSlot.kSlot0)
        (
            config.closedLoop
            # .IMaxAccum(0.03, slot=slot0)
            # .IZone(3, slot=slot0)
            .pid(p=self._constants.proportional_coefficient,  # Slot 0 for position control
                 i=self._constants.integral_coefficient,
                 d=self._constants.derivative_coefficient,
                 slot=slot0)
            .positionWrappingEnabled(True)
            .outputRange(-1, 1)
        )
        # Apply any optional config
        if self._constants.velocity_feedforward is not None:
             config.closedLoop.velocityFF(self._constants.velocity_feedforward, slot=slot0)

        if self._constants.imax_accum is not None:
            config.closedLoop.IMaxAccum(self._constants.imax_accum, slot=slot0)

        if self._constants.izone is not None:
            config.closedLoop.IZone(self._constants.izone, slot=slot0)

        # Set the encoder to return its position in radians
        config.encoder.positionConversionFactor(2 * math.pi)

        return config

    def _check_is_connected(self, config_status: REVLibError | None) -> bool:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        match self._controller_type:
            case ControllerType.SparkFlex | ControllerType.SparkMax:
                version = self._motor.getFirmwareVersion()
                logger.info(f"{self.getName()} firmware version: {version}")

                ok = (version != 0 and (config_status is None or
                                        config_status == REVLibError.kOk)) or \
                     self._is_simulation

                if not ok:
                    logger.warning(f"{self.getName()} firmware version: {version}, status: {config_status}")
                return ok

            case _:
                raise NotImplementedError("Unsupported controller type}")

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results
        """
        return self._is_connected or self._is_simulation

    @property
    def velocity_in_rps(self) -> radians_per_second:
        rps = self._encoder.getVelocity()
        return -rps if self._inverted else rps

    @property
    def position(self) -> radians:
        return self._encoder.getPosition()

    def _set_velocity_goal(self, rpm: revolutions_per_minute, rpm_tolerance: revolutions_per_minute | None) -> None:
        self._velocity_tolerance = rpm_tolerance or 0.0
        self._velocity_goal, previous = max(0.0, min(self._constants.max_rpm, abs(rpm))), self._velocity_goal

        if self._velocity_goal != previous or self._velocity_tolerance != self.tolerance:
            logger.info(f"{self.getName()}: Setting goal RPM to {self._velocity_goal}. previous: {previous}")
            logger.info(
                f"{self.getName()}: current PID controller setpoint before command: {self._pid_controller.getSetpoint()}")

            self._pid_controller.setSetpoint(self._velocity_goal, SparkBase.ControlType.kVelocity)

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        inputs.mechanism_connected = self.is_connected

        # TODO: Is position really important (maybe for playback?)
        inputs.mechanism_position = self.position
        inputs.mechanism_velocity = self.velocity_in_rps
        inputs.mechanism_applied_voltage = self._motor.getAppliedOutput()
        inputs.mechanism_supply_current = self._motor.getOutputCurrent()

    def fault_detection(self, state: str, clear: Optional[bool] = True, notify: Optional[bool] = True) -> None:
        """
        This routine is responsible for reading any existing faults and based
        input parameters, report them for display, and possibly clear them

        All faults detected always results in a warning log message, so please be
        aware of this if you do not clear them
        """
        # For Rev Robotics, the faults are a bitmask
        handle_faults(self.getName(), state, self._motor, clear=clear, notify=notify)
