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

from commands2 import Subsystem
from commands2.button import Trigger
from pykit.networktables.loggeddashboardchooser import LoggedDashboardChooser
from rev import PersistMode, ResetMode, SparkBase, SparkBaseConfig, SparkMax, \
    SparkMaxSim, SparkRelativeEncoderSim
from wpilib import RobotBase, SmartDashboard
from wpimath._controls._controls.plant import DCMotor
from wpimath.units import revolutions_per_minute, seconds

from lib_6107.util.rev_utils import try_until_ok


# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class ShooterConstants:
    MAX_RPM =  5676
    FF = 18.5 / 10000
    PROPORTIONAL_GAIN = 0.5 / 10000
    DERIVATIVE_GAIN = 0.0 / 10000


class RevShooter(Subsystem):
    """
    Rev NEO 21-1650

    """
    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        super().__init__()
        # TODO: add pykit io support

        self._container = container
        self._robot = container.robot
        self._period: seconds = container.robot.getPeriod()
        self._device_id = can_device_id
        self._inverted = inverted
        self._physics_controller = None

        self._velocity_goal: revolutions_per_minute = 0
        self._velocity_tolerance: revolutions_per_minute = 0
        self._current_rpm: revolutions_per_minute = 0

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
            self._sim_encoder = SparkRelativeEncoderSim(self._motor)

        # TODO: Remove following once all works
        self._enable_chooser = LoggedDashboardChooser("Shooter Enable")
        self._enable_chooser.setDefaultOption("False", False)
        self._enable_chooser.addOption("True", True)
        # SmartDashboard.putData("Shooter Enabled", self._enable_chooser)

    @property
    def enabled(self) -> bool:
        return self._enable_chooser.getSelected()

    @property
    def subsystem_trigger(self) -> Trigger:
        return Trigger(lambda: self.enabled)

    @staticmethod
    def _motor_config(inverted: bool) -> SparkBaseConfig:
        config = SparkBaseConfig()
        config.inverted(inverted)
        config.setIdleMode(SparkBaseConfig.IdleMode.kCoast)
        config.limitSwitch.forwardLimitSwitchEnabled(False)
        config.limitSwitch.reverseLimitSwitchEnabled(False)
        config.closedLoop.pid(ShooterConstants.PROPORTIONAL_GAIN, 0.0, ShooterConstants.DERIVATIVE_GAIN)
        config.closedLoop.velocityFF(ShooterConstants.FF)
        config.closedLoop.outputRange(-1, +1)
        return config

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

        self._pid_controller.setReference(self._velocity_goal, SparkBase.ControlType.kVelocity)

    def stop(self) -> None:
        self._motor.stopMotor()

        self._velocity_tolerance = 0
        self._velocity_goal = 0

    def periodic(self) -> None:
        self._current_rpm = self.velocity

        # Update SmartDashboard for this subsystem at a rate slower than the period
        counter = self._robot.counter
        if counter % 100 == 0 or (self._robot.counter % 19 == 0 and
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

        # TODO: Anything

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
        pass
