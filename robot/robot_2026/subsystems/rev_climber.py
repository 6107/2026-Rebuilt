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
from typing import Optional

from commands2 import cmd, Command, Subsystem
from commands2.button import Trigger
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from rev import PersistMode, ResetMode, SparkBase, SparkMax, SparkMaxConfig, SparkMaxSim, SparkRelativeEncoder, \
    SparkRelativeEncoderSim
from wpilib import RobotBase, SendableChooser, SmartDashboard
from wpilib.simulation import BatterySim, RoboRioSim
from wpilib.simulation import ElevatorSim
from wpilib.sysid import State
from wpimath._controls._controls.plant import DCMotor
from wpimath.controller import PIDController
from wpimath.units import amperes, inches, inchesToMeters, kilograms, meters, revolutions_per_minute, seconds, volts

from lib_6107.subsystems.pykit.rotation_mechanism_io import RotationMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)


class ClimberConstants:
    TARGET_RPM: revolutions_per_minute = 10
    PROPORTIONAL_COEFFICIENT = 0.1  # kP
    INTEGRAL_COEFFICIENT = 0.1  # kI
    DERIVATIVE_COEFFICIENT = 0.0  # kD
    LIMIT_CURRENT: amperes = 35
    IZONE_RANGE = 0.0
    GEAR_RATIO = 25.0
    DRIVE_VOLTAGE: volts = 0.10  # Start at 10% power
    SPOOL_DIAMETER: meters = inchesToMeters(1.0)  # TODO: Use an algorythm to compensate for cord already spooled in
    CLIMBER_MIN_HEIGHT: meters = 0.0
    CLIMBER_MAX_HEIGHT: meters = inchesToMeters(9.0)  # TODO: Guess for now
    CLIMBER_TOLERANCE: meters = 0.05  # 5 cm

    #################################################################################
    # Simulation Support
    CARRIAGE_MASS: kilograms = 1.0  # The part that extends up


@autologgable_output
class RevClimber(Subsystem, RotationMechanismIO):

    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
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
        try_until_ok("Climber", 5,
                     lambda: self._motor.configure(self._motor_config(self._inverted),
                                                   ResetMode.kResetSafeParameters,
                                                   PersistMode.kNoPersistParameters))
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
        self._pid_controller: PIDController = PIDController(ClimberConstants.PROPORTIONAL_COEFFICIENT,
                                                            ClimberConstants.INTEGRAL_COEFFICIENT,
                                                            ClimberConstants.DERIVATIVE_COEFFICIENT)
        self._pid_controller.setIZone(ClimberConstants.IZONE_RANGE)

        # The critical attributes/properties for operation
        self._position_goal: inches = 0.0
        self._applied_voltage: volts = 0.0

        # TODO: Remove following once all works
        self._enable_chooser = SendableChooser()
        self._enable_chooser.setDefaultOption("False", False)
        self._enable_chooser.addOption("True", True)
        SmartDashboard.putData("Climber Enabled", self._enable_chooser)

    @property
    def enabled(self) -> bool:
        return self._enable_chooser.getSelected()

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkMaxConfig:
        config = SparkMaxConfig()
        config.inverted(inverted)
        config.setIdleMode(SparkMaxConfig.IdleMode.kBrake)      # Set idle mode as brake
        config.limitSwitch.forwardLimitSwitchEnabled(False)
        config.limitSwitch.reverseLimitSwitchEnabled(False)
        config.smartCurrentLimit(ClimberConstants.LIMIT_CURRENT)

        # Set the position conversion factor (e.g., to convert rotations to inches or meters)
        # The native unit is rotations. For a 1-inch spool, the factor to get inches would be (math.pi * 1.0)
        config.encoder.positionConversionFactor(math.pi * ClimberConstants.SPOOL_DIAMETER)

        # Set the velocity conversion factor (e.g., to convert RPM to inches/second)
        # The native unit is RPM. So to go to inches / second use:
        #    inches/rev) / 60 seconds
        velocity_factor = (ClimberConstants.SPOOL_DIAMETER / math.pi) / (ClimberConstants.GEAR_RATIO * 60.0)

        config.encoder.velocityConversionFactor(velocity_factor)
        return config

    def reset(self) -> None:
        self._motor.stopMotor()
        self._position_goal = 0
        self._encoder.setPosition(0.0)
        self._pid_controller.reset()

    def stop(self, brake: bool) -> None:
        self._pid_controller.setSetpoint(self.position)
        self._motor.stopMotor()

        if brake:
            # TODO: Anything else to brake?
            pass

    @property
    def closed_loop(self) -> bool:
        return self._closed_loop

    def set_closed_loop(self, closed_loop: bool) -> None:
        self._closed_loop = closed_loop

    @property
    def position(self) -> inches:
        return self._inputs.mechanism_position

    @position.setter
    def position(self, position: inches) -> None:
        if self._position_goal != position:
            self._position_goal = position
            self._pid_controller.setSetpoint(position)

    def at_min(self) -> bool:
        return self._inputs.mechanism_position <= ClimberConstants.CLIMBER_MIN_HEIGHT + \
            ClimberConstants.CLIMBER_TOLERANCE

    def at_max(self) -> bool:
        return self._inputs.mechanism_position >= ClimberConstants.CLIMBER_MAX_HEIGHT - \
            ClimberConstants.CLIMBER_TOLERANCE

    def stop(self, brake: Optional[bool] = True) -> None:
        self._motor.stopMotor()

        if brake:
            # TODO: Anything else to brake?
            pass

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        # TODO: Anything

    def simulationPeriodic(self, **kwargs) -> None:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function. It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called.

        To unify the two uses, our call signature above has a kwargs parameter so we
        know when we are being called. Typically, we only need to support one method
        but for future simulation purposes, if called with keywords, return the amperage
        used in this interval
        """
        if self._robot.isEnabled():
            self._sim_climber.setInputVoltage(self._sim_motor.getAppliedOutput())
            self._sim_climber.update(self._period)

            # Set the simulated encoder
            self._sim_encoder.setPosition(self._sim_climber.getPosition())

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._sim_climber.getCurrentDraw()]))

    def periodic(self) -> None:
        LogTracer.resetOuter("ClimberSubsystem periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Climber", self._inputs)
        LogTracer.record("UpdateInputs")

        if self._closed_loop:
            self.set_position(self._position_goal)

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Climber/goal", self._position_goal)
        Logger.recordOutput("Climber/ClosedLoop", self._closed_loop)
        LogTracer.recordTotal()

        # Update SmartDashboard for this subsystem at a rate slower than the period
        counter = self._robot.counter
        if counter % 100 == 0 or (self._robot.counter % 7 == 0 and
                                  self._robot.isEnabled()):
            self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        pass

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        SmartDashboard.putNumber("Climber/position", self.position)
        SmartDashboard.putNumber("Climber/goal", self._position_goal)
        SmartDashboard.putNumber("Climber/speed", self._inputs.mechanism_speed)
        SmartDashboard.putBoolean("Climber/closed-loop", self._closed_loop)

    def updateInputs(self, inputs: RotationMechanismIO.RotationMechanismIOInputs) -> None:
        inputs.mechanism_connected = True   # TODO: Figure this one out
        inputs.mechanism_position = self._encoder.getPosition()
        inputs.mechanism_speed = self._encoder.getVelocity()

        inputs.mechanism_applied_voltage = self._motor.getBusVoltage()
        inputs.mechanism_supply_current = self._motor.getOutputCurrent()
        # TODO: Figure this out or drop it inputs.mechanism_torque_amps = self._motor.get

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
