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
from rev import PersistMode, ResetMode, SparkBase, SparkBaseConfig, SparkMax
from wpilib import SendableChooser, SmartDashboard
from wpimath.controller import PIDController
from wpimath.units import amperes, revolutions_per_minute


# TODO: Move following to constant
# TODO: Run the REV Hardware Client and come up with our numbers
#

class ClimberConstants:
    TARGET_RPM: revolutions_per_minute = 10
    PROPORTIONAL_COEFFICIENT = 0.1     # kP
    INTEGRAL_COEFFICIENT = 0.1         # kI
    DERIVATIVE_COEFFICIENT = 0.0       # kD
    LIMIT_CURRENT: amperes = 35
    IZONE_RANGE = 0.0

class RevClimber(Subsystem):

    def __init__(self, container: 'RobotContainer', can_device_id: int, inverted: bool) -> None:
        super().__init__()
        # TODO: add pykit io support

        self._container = container
        self._robot = container.robot
        self._device_id = can_device_id
        self._inverted = inverted

        # Set up the motor controller
        self._motor = SparkMax(can_device_id, SparkBase.MotorType.kBrushless)
        self._motor.configure(self._motor_config(self._inverted),
                              ResetMode.kResetSafeParameters,
                              PersistMode.kPersistParameters)

        self._encoder = self._motor.getEncoder()
        self._encoder.setPosition(0.0)
        self._position: float = 0.0
        self._position_goal: float = 0.0
        self._pid_controller: PIDController = PIDController(ClimberConstants.PROPORTIONAL_COEFFICIENT,
                                                            ClimberConstants.INTEGRAL_COEFFICIENT,
                                                            ClimberConstants.DERIVATIVE_COEFFICIENT)
        self._pid_controller.setIZone(ClimberConstants.IZONE_RANGE)

        # TODO: How do we choose the speed/RPM
        # self._target_rpm = ClimberConstants.TARGET_RPM

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
    def _motor_config(inverted: bool) -> SparkBaseConfig:
        config = SparkBaseConfig()
        config.inverted(inverted)
        config.setIdleMode(SparkBaseConfig.IdleMode.kBrake)      # Set idle mode as brake
        config.limitSwitch.forwardLimitSwitchEnabled(False)
        config.limitSwitch.reverseLimitSwitchEnabled(False)
        config.smartCurrentLimit(ClimberConstants.LIMIT_CURRENT)
        return config

    def periodic(self) -> None:
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

    def reset(self) -> None:
        self._motor.stopMotor()
        self._position = 0
        self._position_goal = 0
        self._encoder.setPosition(0.0)
        self._pid_controller.reset()

    @property
    def position(self) -> float:
        return self._encoder.getPosition()

    @position.setter
    def position(self, position: float) -> None:
        if self._position_goal != position:
            self._position_goal = position
            self._pid_controller.setSetpoint(position)

    def periodic(self) -> None:
        self._position = self._encoder.getPosition()

    def stop(self, brake: bool) -> None:
        self._pid_controller.setSetpoint(self.position)
        self._motor.stopMotor()

        if brake:
            # TODO: Anything else to brake?
            pass
