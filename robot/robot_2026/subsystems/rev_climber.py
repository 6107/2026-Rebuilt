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

from commands2 import cmd, Command, Subsystem
from commands2.sysid import SysIdRoutine
from rev import ClosedLoopSlot, PersistMode, ResetMode, REVLibError, SparkBase, SparkLowLevel, SparkMax, SparkMaxConfig, \
    SparkMaxSim, SparkRelativeEncoder, SparkRelativeEncoderSim
from wpilib import Color, Color8Bit, Mechanism2d, MechanismLigament2d, MechanismRoot2d, RobotBase, RobotController, \
    SmartDashboard
from wpilib.simulation import BatterySim, ElevatorSim, RoboRioSim
from wpilib.sysid import State
from wpimath.system.plant import DCMotor
from wpimath.units import amperes, inches, inchesToMeters, kilograms, meters, revolutions_per_minute, seconds, volts

from lib_6107.pykit.logger import Logger
from lib_6107.pykit.networktables.loggednetworkboolean import LoggedNetworkBoolean
from lib_6107.subsystems.pykit.rotation_mechanism_io import RotationMechanismIO
from lib_6107.util.competition import event_active
from lib_6107.util.rev_utils import handle_faults, try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


class ClimberConstants:
    TARGET_RPM: revolutions_per_minute = 10

    PROPORTIONAL_COEFFICIENT = 1.0    # kP
    INTEGRAL_COEFFICIENT     = 0.0  # kI
    DERIVATIVE_COEFFICIENT   = 0.2  # kD

    LIMIT_CURRENT: amperes = 35

    GEAR_RATIO = 25.0
    DRIVE_INPUT_MIN: float = -0.3  # [-1.0 .. 1.0]
    DRIVE_INPUT_MAX: float = 1.0  # [-1.0 .. 1.0]

    SPOOL_DIAMETER: inches = 1.25

    # The Actual Height is about 8.375 inches. We start with the climber retracted to the 8" mark
    # and since we are using the internal encoder, turning that on will set the encoder to 0 at that
    # offset. The position compensation factor is set so that 5" hits the 5" mark, but to go the
    # full 8", the setpoint below is used (so it is not in inches)
    CLIMBER_POS_FACTOR = 2.25        # TODO: Need to look this up with Rev Client 2.0

    CLIMBER_MIN_HEIGHT: inches = 0.0
    CLIMBER_MAX_HEIGHT: inches = 8.375

    CLIMBER_RETRACTED_SETPOINT = 0.0    # Fully retracted to the 8" mark
    CLIMBER_EXTENDED_SETPOINT = -12.833 # Fully extended with a slightly snug line
    CLIMBER_TOLERANCE = 0.75

    CLIMBER_ROOT_X: meters = inchesToMeters(.5)  # Bottom left corner of robot is (0, 0)
    CLIMBER_ROOT_Y: meters = inchesToMeters(12.5)
    CLIMBER_BASE_LENGTH: meters = inchesToMeters(8.0)  # TODO: Validate all this locations/lengths
    CLIMBER_UPPER_MIN_LENGTH: meters = inchesToMeters(2.0)

    #################################################################################
    # Simulation Support
    CARRIAGE_MASS: kilograms = 1.0  # The part that extends up

#@autologgable_output
class RevClimber(Subsystem, RotationMechanismIO):

    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        self._initialized = False

        Subsystem.__init__(self)
        RotationMechanismIO.__init__(self, "Climber")

        # General attributes
        self.setName(self.__class__.__name__)
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._closed_loop = True        # Autonomous runs as a closed loop
        self._inputs = RotationMechanismIO.RotationMechanismIOInputs()
        self._physics_controller = None

        # Set up the motor controller
        self._motor = SparkMax(can_device_id, SparkBase.MotorType.kBrushless)
        status = try_until_ok("Climber", 5,
                              lambda: self._motor.configure(self._motor_config(self._inverted),
                                                            ResetMode.kResetSafeParameters,
                                                            PersistMode.kNoPersistParameters))
        self._is_connected = self._check_is_connected(status)

        # Set up the encoder
        self._encoder: SparkRelativeEncoder = self._motor.getEncoder()
        self._encoder.setPosition(0.0)

        # Support simulation
        if RobotBase.isSimulation():
            motor_model = DCMotor.NEO(1)
            self._sim_motor = SparkMaxSim(self._motor, motor_model)
            self._sim_encoder = SparkRelativeEncoderSim(self._motor)
            self._sim_climber = ElevatorSim(motor_model,
                                            ClimberConstants.GEAR_RATIO,
                                            ClimberConstants.CARRIAGE_MASS,
                                            ClimberConstants.SPOOL_DIAMETER,
                                            ClimberConstants.CLIMBER_MIN_HEIGHT,
                                            ClimberConstants.CLIMBER_MAX_HEIGHT,
                                            True,  # Simulate gravity
                                            0.0,  # Staring Height
                                            [0.01, 0.0])

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._pid_controller = self._motor.getClosedLoopController()

        # The critical attributes/properties for operation
        self._position_goal: float = ClimberConstants.CLIMBER_RETRACTED_SETPOINT
        self._applied_voltage: volts = 0.0

        #####################################
        # Visualization support
        self._mech_2d: Mechanism2d = Mechanism2d(inchesToMeters(20), inchesToMeters(50))
        mech_root: MechanismRoot2d = self._mech_2d.getRoot("Climber Root",
                                                     ClimberConstants.CLIMBER_ROOT_X,
                                                     ClimberConstants.CLIMBER_ROOT_Y)
        mech_base: MechanismLigament2d = mech_root.appendLigament("Climber Base",
                                                                  ClimberConstants.CLIMBER_BASE_LENGTH,
                                                                  90,
                                                                  color=Color8Bit(Color.kBlue))
        self._mech_upper: MechanismLigament2d = mech_base.appendLigament("Climber Upper",
                                                                         ClimberConstants.CLIMBER_UPPER_MIN_LENGTH,
                                                                         0,
                                                                         color=Color8Bit(Color.kYellow))
        SmartDashboard.putData("Climber-mech", self._mech_2d)

        self._enable_chooser = LoggedNetworkBoolean("Climber/Enabled",
                                                    defaultValue=event_active())
        self._initialized = True

    @property
    def is_connected(self) -> bool:
        """
        Detect if this device is connected to the CAN Bus.  For Rev Robotics,
        the default way is based on config results. When we support CTRE, they
        have a 'isStatusOK' call that is useful.
        """
        return self._is_connected

    def _check_is_connected(self, status: REVLibError | None) -> bool:
        """
        For Rev Robotics, the only way to check if all is well i
        """
        version = self._motor.getFirmwareVersion()

        logger.info(f"{self.getName()} firmware version: {version}")

        ok = (version != 0 and (status is None or status == REVLibError.kOk)) or RobotBase.isSimulation()

        if not ok:
            logger.warning(f"{self.getName()} firmware version: {version}, status: {status}")

        return ok

    @property
    def is_initialized(self) -> bool:
        return self._initialized

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self.is_initialized and self._enable_chooser.value

    @staticmethod
    def _motor_config(inverted: bool) -> SparkMaxConfig:

        config = (
            SparkMaxConfig()
            .inverted(inverted)
            .setIdleMode(SparkMaxConfig.IdleMode.kBrake)  # Set idle mode as brake
            .smartCurrentLimit(ClimberConstants.LIMIT_CURRENT)
        )
        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

        # Following is set from experimentation since we have a cloth cord that
        # stretches and the diameter increases as more cord is spooled in
        config.encoder.positionConversionFactor(ClimberConstants.CLIMBER_POS_FACTOR)

        # Set the velocity conversion factor (e.g., to convert RPM to inches/second)
        # The native unit is RPM. So to go to inches / second use:
        #    inches/rev) / 60 seconds.  Value is: 0.0002122065907891938
        #velocity_factor = (ClimberConstants.SPOOL_DIAMETER / math.pi) / (ClimberConstants.GEAR_RATIO * 60.0)
        #config.encoder.velocityConversionFactor(velocity_factor)

        slot0 = ClosedLoopSlot(ClosedLoopSlot.kSlot0)
        (
            config.closedLoop
            # .IMaxAccum(0.03, slot=slot0)
            # .IZone(3, slot=slot0)
            .pid(p=ClimberConstants.PROPORTIONAL_COEFFICIENT,  # Slot 0 for position control
                 i=ClimberConstants.INTEGRAL_COEFFICIENT,
                 d=ClimberConstants.DERIVATIVE_COEFFICIENT,
                 slot=slot0)
            .outputRange(ClimberConstants.DRIVE_INPUT_MIN,
                         ClimberConstants.DRIVE_INPUT_MAX,
                         slot=slot0)
        )
        return config

    def reset(self) -> None:
        self._motor.stopMotor()
        self._position_goal = 0.0

        self._mech_upper.setLength(ClimberConstants.CLIMBER_MIN_HEIGHT)
        raise NotImplementedError("climber reset: Reset command was called but is not supported at this time")

    def stop(self) -> None:
        self._motor.stopMotor()

    @property
    def closed_loop(self) -> bool:
        return self._closed_loop

    def set_closed_loop(self, closed_loop: bool) -> None:
        self._closed_loop = closed_loop

    @property
    def position(self) -> float:
        return self._inputs.mechanism_position

    @property
    def position_meters(self) -> float:
        return inches(self.position)

    @position.setter
    def position(self, position: float) -> None:
        if self._position_goal != position:
            self._position_goal = position
            self._pid_controller.setSetpoint(position, SparkLowLevel.ControlType.kPosition,
                                             ClosedLoopSlot(ClosedLoopSlot.kSlot0))
            logger.info(f"Climber position set to {position}")

    def at_min(self) -> bool:
        return self._inputs.mechanism_position <= ClimberConstants.CLIMBER_MIN_HEIGHT + \
            ClimberConstants.CLIMBER_TOLERANCE

    def at_max(self) -> bool:
        return self._inputs.mechanism_position >= ClimberConstants.CLIMBER_MAX_HEIGHT - \
            ClimberConstants.CLIMBER_TOLERANCE

    def extend(self) -> None:
        logger.info(f"Climber Extend: Setting position to {ClimberConstants.CLIMBER_EXTENDED_SETPOINT}")
        self.position = ClimberConstants.CLIMBER_EXTENDED_SETPOINT

    def retract(self) -> None:
        logger.info(f"Climber Retract: Setting position to {ClimberConstants.CLIMBER_RETRACTED_SETPOINT}")
        self.position = ClimberConstants.CLIMBER_RETRACTED_SETPOINT

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller

    def simulationPeriodic(self) -> None:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function). It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called. One other
        'important' difference is 'update_sim' is called at a period >= 10 ms instead
        of the default 20 mS for the CommandScheduler's simulationPeriodic (this function).
        """
        if self._robot.isEnabled() and self.is_initialized:
            LogTracer.resetOuter(f"{self.getName()}-simulationPeriodic")

            voltage = self._sim_motor.getAppliedOutput() * RobotController.getBatteryVoltage()

            self._sim_climber.setInputVoltage(voltage)
            self._sim_climber.update(self._period)

            # Set the simulated encoder
            self._sim_encoder.setPosition(self._sim_climber.getPosition())

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._sim_climber.getCurrentDraw()]))
            LogTracer.recordTotal()

    # def update_sim(self, now: float, tm_diff: float) -> None:
    #     """
    #     Called when the simulation parameters for the program need to be updated.
    #     This function is called from the '_simulationPeriodic' function of the
    #     robotpy core routine and is called at a period >= 10 mS. Note that the
    #     CommandScheduler also has an 'simulationPeriodic' function that it calls
    #     into all Command2 based subsystems at its update period which has a
    #     default rate of 20 mS.
    #
    #     This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
    #     that function uses pykit's logging method, you should use those values in
    #     your simulation.
    #
    #     :param now:     The current time as a float
    #     :param tm_diff: The amount of time that has passed since the last
    #                     time that this function was called
    #     """
    #     if not self.is_initialized:
    #         return

    def periodic(self) -> None:
        if not self.is_initialized:
            return

        LogTracer.resetOuter("ClimberSubsystem periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Climber", self._inputs)
        LogTracer.record("UpdateInputs")

        # Update visualization
        self._mech_upper.setLength(ClimberConstants.CLIMBER_MIN_HEIGHT + self.position_meters)

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Climber/goal", self._position_goal)
        # Logger.recordOutput("Climber/ClosedLoop", self._closed_loop)
        LogTracer.recordTotal()

        # # Update SmartDashboard for this subsystem at a rate slower than the period
        # counter = self._robot.counter
        # if counter % 100 == 0 or (self._robot.counter % 37 == 0 and
        #                           self._robot.isEnabled()):
        #     self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        # SmartDashboard.putNumber("Climber/position", 0.0)
        # SmartDashboard.putNumber("Climber/goal", 0.0)
        # SmartDashboard.putNumber("Climber/speed", 0.0)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        # SmartDashboard.putNumber("Climber/position", self.position)
        # SmartDashboard.putNumber("Climber/goal", self._position_goal)
        # SmartDashboard.putNumber("Climber/speed", self._inputs.mechanism_speed)
        # SmartDashboard.putBoolean("Climber/closed-loop", self._closed_loop)

    def updateInputs(self, inputs: RotationMechanismIO.RotationMechanismIOInputs) -> None:
        if not self.is_initialized:
            return

        inputs.mechanism_connected = self.is_connected
        inputs.mechanism_position = self._encoder.getPosition()
        inputs.mechanism_speed = self._encoder.getVelocity()

        inputs.mechanism_applied_voltage = self._motor.getBusVoltage()
        inputs.mechanism_supply_current = self._motor.getOutputCurrent()
        # TODO: Figure this out or drop it -> inputs.mechanism_torque_amps = self._motor.get

    def set_position(self, position: inches) -> None:
        """
        Set the desired encoder position. This is primarily for Autonomous mode when
        we are running with a closed loop system

        Args:
            position (rotations +/-): The desired number of rotations
        """
        # Limit to max/min
        position = max(min(position, ClimberConstants.CLIMBER_MAX_HEIGHT),
                       ClimberConstants.CLIMBER_MIN_HEIGHT)
        self._encoder.setPosition(position)

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        self._motor.setVoltage(voltage)

    def sys_id_routine(self, subsystem: Subsystem) -> Command:
        """
        Model the behavior of the climber (for better control) by sweeping through the max and min heights.
        """
        def logState(state: State) -> None:
            logged_state = ""
            match state:
                case State.kQuasistaticForward:
                    logged_state = "quasistatic-forward"
                case State.kQuasistaticReverse:
                    logged_state = "quasistatic-reverse"
                case State.kDynamicForward:
                    logged_state = "dynamic-forward"
                case State.kDynamicReverse:
                    logged_state = "dynamic-reverse"
                case State.kNone:
                    logged_state = "none"
            Logger.recordOutput("Climber/SysID State", logged_state)

        characterization_routine = SysIdRoutine(SysIdRoutine.Config(0.5, 6, 10, logState),
                                                SysIdRoutine.Mechanism(self.set_voltage,
                                                                       (lambda _: None),
                                                                       subsystem,
                                                                       "Climber"))
        return cmd.sequence(
            cmd.runOnce(lambda: self.set_closed_loop(False), self),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kForward).until(self.at_max),
            characterization_routine.quasistatic(SysIdRoutine.Direction.kReverse).until(self.at_min),
            characterization_routine.dynamic(SysIdRoutine.Direction.kForward).until(self.at_max),
            characterization_routine.dynamic(SysIdRoutine.Direction.kReverse).until(self.at_min),
            cmd.runOnce(lambda: self.set_closed_loop(True), self),
        )

    def fault_detection(self, state: str, clear: Optional[bool] = True, notify: Optional[bool] = True) -> None:
        """
        This routine is responsible for reading any existing faults and based
        input parameters, report them for display, and possibly clear them

        All faults detected always results in a warning log message, so please be
        aware of this if you do not clear them

        TODO: Good thing for a base class, don't you think
        """
        # For Rev Robotics, the faults are a bitmask
        handle_faults("Climber", state, self._motor, clear=clear, notify=notify)
