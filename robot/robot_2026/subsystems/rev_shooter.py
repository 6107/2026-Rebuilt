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
from rev import PersistMode, ResetMode, SparkBase, SparkBaseConfig, SparkMax
from wpilib import SmartDashboard
from wpimath.units import revolutions_per_minute


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
        self._device_id = can_device_id
        self._inverted = inverted

        self._velocity_goal: revolutions_per_minute = 0
        self._velocity_tolerance: revolutions_per_minute = 0
        self._current_rpm: revolutions_per_minute = 0

        # Set up the motor controller
        self._shooter_motor = SparkMax(can_device_id, SparkBase.MotorType.kBrushless)
        self._shooter_motor.configure(self._motor_config(self._inverted),
                                      ResetMode.kResetSafeParameters,
                                      PersistMode.kPersistParameters)

        self._pid_controller = self._shooter_motor.getClosedLoopController()
        self._encoder = self._shooter_motor.getEncoder()

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

    def periodic(self) -> None:
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

    @property
    def not_ready(self) -> str:
        velocity = self.velocity
        if velocity < self._velocity_goal - self._velocity_tolerance:
            return f"shooter under velocity goal: {velocity} < {self._velocity_goal}"

        if velocity > self._velocity_goal + self._velocity_tolerance:
            return f"shooter above velocity goal: {velocity} > {self._velocity_goal}"

        return ""  # shooter is ready

    @property
    def velocity(self) -> revolutions_per_minute:
        rpm = self._encoder.getVelocity()
        return -rpm if self._inverted else rpm

    def periodic(self) -> None:
        self._current_rpm = self.velocity

    def set_velocity_goal(self, rpm: int, rpm_tolerance) -> None:
        self._velocity_tolerance = rpm_tolerance
        self._velocity_goal = max(0, min(ShooterConstants.MAX_RPM, abs(rpm)))

        self._pid_controller.setReference(self._velocity_goal, SparkBase.ControlType.kVelocity)

    def stop(self) -> None:
        self._shooter_motor.stopMotor()

        self._velocity_tolerance = 0
        self._velocity_goal = 0
