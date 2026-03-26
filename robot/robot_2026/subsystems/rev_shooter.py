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

from commands2 import PrintCommand, Subsystem
from commands2.button import Trigger
from commands2.command import Command
from commands2.sysid import SysIdRoutine
from pykit.autolog import autologgable_output
from pykit.logger import Logger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from rev import ClosedLoopSlot, PersistMode, ResetMode, SparkBase, SparkBaseConfig, SparkClosedLoopController, SparkMax, \
    SparkMaxConfig, SparkMaxSim
from wpilib import RobotBase, SendableChooser, SmartDashboard
from wpilib.simulation import BatterySim, RoboRioSim
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import amperes, radiansPerSecondToRotationsPerMinute, revolutions_per_minute, seconds, volts

from lib_6107.subsystems.pykit.rpm_mechanism_io import RpmMechanismIO
from lib_6107.util.rev_utils import try_until_ok
from robot_2026.util.logtracer import LogTracer

logger = logging.getLogger(__name__)

# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class ShooterConstants:
    FF = 18.5 / 10000
    PROPORTIONAL_COEFFICIENT = 0.5 / 10000  # kP
    INTEGRAL_COEFFICIENT = 0  # kI
    DERIVATIVE_COEFFICIENT = 0.0 / 10000

    LIMIT_CURRENT: amperes = 40

    GEAR_REDUCTION = 6.75  # TODO: Get number
    MEASUREMENT_STD_DEV = [0.0, 0.0]  # TODO: Get number for noise
    MAX_RPM = 5676


@autologgable_output
class RevShooter(Subsystem, RpmMechanismIO):
    """
    Rev NEO 21-1650
    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        Subsystem.__init__(self)
        RpmMechanismIO.__init__(self, "Shooter")

        # General attributes
        self.setName(self.__class__.__name__)
        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._closed_loop = True  # Autonomous runs as a closed loop (only false
        self._inputs = RpmMechanismIO.RpmMechanismIOInputs()

        self._physics_controller = None

        # Set up the motor controller
        self._motor = SparkMax(can_device_id, SparkBase.MotorType.kBrushless)
        try_until_ok("Shooter", 5,
                     lambda: self._motor.configure(self._motor_config(self._inverted),
                                                   ResetMode.kResetSafeParameters,
                                                   PersistMode.kNoPersistParameters))
        self._pid_controller = self._motor.getClosedLoopController()
        self._encoder = self._motor.getEncoder()

        # Support simulation
        if RobotBase.isSimulation():
            self._sim_motor = SparkMaxSim(self._motor, DCMotor.NEO(1))

        # PID Controller for use while in autonomous mode. During teleop end-game, the
        # operator or shooter's controller will have manual up/down control.
        self._pid_controller: SparkClosedLoopController = self._motor.getClosedLoopController()
        self._pid_controller.setSetpoint(0, SparkBase.ControlType.kVoltage, ClosedLoopSlot(0))

        # The critical attributes/properties for operation
        self._rpm_goal: revolutions_per_minute = 0.0
        self._applied_voltage: volts = 0.0

        self._velocity_goal: revolutions_per_minute = 0
        self._velocity_tolerance: revolutions_per_minute = 0  # TODO: Reconcile across devices
        self._current_rpm: revolutions_per_minute = 0

        # # TODO:  Next is not currently supported
        self._sysid_routine = None
        # self._sysid_routine = SysIdRoutine(SysIdRoutine.Config(),
        #                                    SysIdRoutine.Mechanism(self.log_motor,
        #                   lambda voltage: self._motor.setVoltage(voltage),
        #                                                           self))

        # TODO: Remove following once all works
        # self._enable_chooser = LoggedDashboardChooser("Shooter Enabled")
        self._enable_chooser = LoggedDashboardChooser("Shooter Enabled")

        self._enable_chooser.setDefaultOption("False", False)
        self._enable_chooser.addOption("True", True)

        if isinstance(self._enable_chooser, SendableChooser):
            SmartDashboard.putData("Shooter Enabled", self._enable_chooser)
        elif isinstance(self._enable_chooser, LoggedDashboardChooser):
            pass

    def log_motor(self, log):
        log.motor("shooter").field("velocity", self._encoder.getVelocity())

    @property
    def enabled(self) -> bool:
        """
        Returns True if the Chooser Dialog is True, indicating the subsystem is enabled.

        Note: Enabled is different from active. It is primarily used to indicate that it
        can perform its operations.
        """
        return self._enable_chooser.getSelected() is True

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkBaseConfig:
        config = (SparkMaxConfig().
                  inverted(inverted).
                  smartCurrentLimit(ShooterConstants.LIMIT_CURRENT).
                  setIdleMode(SparkMaxConfig.IdleMode.kCoast)
                  )

        config.limitSwitch.forwardLimitSwitchEnabled(False).reverseLimitSwitchEnabled(False)

        # Closed loop configuration parameters, slot=0
        # TODO: Use SysID to determine actual values
        #
        #   P:     If you’re not where you want to be, get there.
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
            # .IMaxAccum(0.03, slot=slot0).
            # .IZone(3, slot=slot0).
            .pid(p=ShooterConstants.PROPORTIONAL_COEFFICIENT,
                 i=0.0,
                 d=ShooterConstants.DERIVATIVE_COEFFICIENT,
                 slot=slot0)
            .outputRange(-1, 1)
            .velocityFF(ShooterConstants.FF)
        )
        return config

    def reset(self) -> None:
        self.stop()
        logger.info(f"Shooter Roller: Reset command was called  *** *** *** ***   NOT YET SUPPORTED")
        self._rpm_goal = 0

    @property
    def velocity(self) -> revolutions_per_minute:
        rpm = self._encoder.getVelocity()
        return -rpm if self._inverted else rpm

    @property
    def not_ready(self) -> str:
        velocity = self.velocity
        if velocity < self._velocity_goal - self._velocity_tolerance:
            return f"shooter under velocity goal: {velocity} < {self._velocity_goal}"

        if velocity > self._velocity_goal + self._velocity_tolerance:
            return f"shooter above velocity goal: {velocity} > {self._velocity_goal}"

        return ""  # shooter is ready

    def set_velocity_goal(self, rpm: int, rpm_tolerance) -> None:
        self._velocity_tolerance = rpm_tolerance
        self._velocity_goal = max(0, min(ShooterConstants.MAX_RPM, abs(rpm)))

        self._pid_controller.setSetpoint(self._velocity_goal, SparkBase.ControlType.kVelocity)

    def set_rpm_goal(self, goal: revolutions_per_minute) -> None:
        if self._rpm_goal != goal:
            logger.info(f"Shooter: Setting goal RPM to {goal}. currently at: {self._encoder.getPosition()}")
            logger.info(
                f"Shooter: current PID controller setpoint before command is: {self._pid_controller.getSetpoint()}")

            self._rpm_goal = goal

            self._pid_controller.setSetpoint(goal, SparkBase.ControlType.kPosition, ClosedLoopSlot(0))
            # TODO: What about velocity goal?
            #       https://docs.revrobotics.com/revlib/spark/closed-loop/getting-started-with-pid-tuning

    def stop(self) -> None:
        self._motor.stopMotor()

        self._velocity_tolerance = 0
        self._velocity_goal = 0

    def periodic(self) -> None:
        self._current_rpm = self.velocity  # TODO: Reconcile

        LogTracer.resetOuter("Intake Roller periodic")

        self.updateInputs(self._inputs)

        Logger.processInputs("Intake Roller", self._inputs)
        LogTracer.record("UpdateInputs")

        LogTracer.record("Closed Loop Control")
        Logger.recordOutput("Intake/Roller/goal", self._rpm_goal)
        Logger.recordOutput("Intake/Roller/closed-loop", self._closed_loop)
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
        SmartDashboard.putNumber("Shooter/rpmGoal", self._velocity_goal)
        SmartDashboard.putNumber("Shooter/rpmCurrent", self._current_rpm)
        # TODO: Reconcile
        SmartDashboard.putNumber("Shooter/goal", self._rpm_goal)
        SmartDashboard.putBoolean("Shooter/closed-loop", self._closed_loop)
        SmartDashboard.putNumber("Shooter/RPM", self._inputs.drive_velocity)
        
    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:

        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        # TODO: Anything

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
        pass

    def updateInputs(self, inputs: RpmMechanismIO.RpmMechanismIOInputs) -> None:
        inputs.drive_connected = True  # TODO: Figure this one out
        inputs.encoder_connected = True  # TODO: Figure this one out
        inputs.drive_velocity = self._encoder.getVelocity()
        inputs.drive_applied = self._motor.getBusVoltage()
        inputs.drive_supply_current = self._motor.getOutputCurrent()

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be updated.
        This function is called from the '_simulationPeriodic' function of the
        robotpy core routine and is called at a period >= 10 mS. Note that the
        CommandScheduler also has an 'simulationPeriodic' function that it calls
        into all Command2 based subsystems at its update period which has a
        default rate of 20 mS.

        This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
        that function uses pykit's logging method, you should use those values in
        your simulation.

        :param now:     The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        if self._robot.isEnabled() and self._sim_motor is not None:
            pass
            voltage = RoboRioSim.getVInVoltage()
            rpm = radiansPerSecondToRotationsPerMinute(self._sim_motor.getVelocity())
            self._sim_motor.iterate(rpm, voltage, tm_diff)

            # And simulate current drain
            RoboRioSim.setVInVoltage(BatterySim.calculate([self._sim_motor.getMotorCurrent()]))

    def set_voltage(self, voltage: volts) -> None:
        """
        Set the drive voltage
        """
        if voltage != self._motor.getAppliedOutput():
            logger.info(f"Intake Indexer: Setting voltage to {voltage}")
            self._motor.setVoltage(voltage)

    def sys_id_routine(self) -> Command:
        """
        Model the behavior of the intake (for better control) by sweeping through
        the max and min heights.
        """
        # TODO: Add to non-competition AUTO chooser
        if self._sysid_routine is not None:
            return self._sysid_routine.quasistatic(SysIdRoutine.Direction.kForward)

        logger.error(f"Intake Indexer: SysID routine is not currently supported")
        return PrintCommand("Intake Indexer: SysID routine is not currently supported")
