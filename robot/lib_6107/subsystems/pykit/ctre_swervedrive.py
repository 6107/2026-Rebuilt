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


from phoenix6 import StatusSignal
from phoenix6.controls import (
    MotionMagicVoltage,
    VelocityTorqueCurrentFOC,
)
from phoenix6.hardware.cancoder import CANcoder
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.swerve.swerve_module import SwerveModule
from pykit.logger import Logger
from wpimath.geometry import Rotation2d
from wpimath.kinematics import SwerveModulePosition, SwerveModuleState
from wpimath.units import meters, meters_per_second

import constants
from lib_6107.subsystems.pykit.swervedrive_io import SwerveModuleIO
from lib_6107.util.phoenix6_signals import Phoenix6Signals
from util.logtracer import LogTracer


class CtreSwerveModule(SwerveModuleIO):

    steer_request: MotionMagicVoltage = MotionMagicVoltage(0)
    drive_request: VelocityTorqueCurrentFOC = VelocityTorqueCurrentFOC(0)

    def __init__(self, module: SwerveModule, name: str):
        super().__init__(name)

        self._module = module
        self._config = module
        self._drive_motor: TalonFX = module.drive_motor
        self._steer_motor: TalonFX = module.steer_motor
        self._encoder: CANcoder = module.encoder

        self._inputs = SwerveModuleIO.SwerveModuleIOInputs()
        self._previous_position = SwerveModulePosition()

        self._wheel_radius: meters = constants.WHEEL_RADIUS
        self._min_wheel_linear_velocity: meters_per_second = constants.MIN_SPEED

        self.prev_position = SwerveModulePosition()

        # update pykit I/O attributes to initial values
        self.drive_position: StatusSignal = self._drive_motor.get_position()
        self.drive_velocity: StatusSignal = self._drive_motor.get_velocity()
        self.drive_applied: StatusSignal = self._drive_motor.get_motor_voltage()
        self.drive_supply_current: StatusSignal = self._drive_motor.get_supply_current()
        self.drive_torque_current: StatusSignal = self._drive_motor.get_torque_current()

        self.turn_position: StatusSignal = self._steer_motor.get_position()
        self.turn_velocity: StatusSignal = self._steer_motor.get_velocity()
        self.turn_applied: StatusSignal = self._steer_motor.get_motor_voltage()
        self.turn_supply_current: StatusSignal = self._steer_motor.get_supply_current()
        self.turn_torque_current: StatusSignal = self._steer_motor.get_torque_current()

        self.turn_absolute_position: StatusSignal = self._encoder.get_absolute_position()

        Phoenix6Signals.register_signals(self.drive_velocity,
                                         self.drive_applied,
                                         self.drive_supply_current,
                                         self.drive_torque_current,
                                         self.turn_position,
                                         self.turn_velocity,
                                         self.turn_applied,
                                         self.turn_supply_current,
                                         self.turn_torque_current,
                                         self.turn_absolute_position)

    def getSwerveAngle(self) -> Rotation2d:
        return Rotation2d(self._inputs.turn_position)

    def setSwerveAngle(self, swerve_angle: Rotation2d) -> None:
        steer_encoder_pulses = swerve_angle.radians() / constants.RADIANS_PER_REVOLUTION
        self._steer_motor.set_position(steer_encoder_pulses)

    def getSwerveEncoderAngle(self) -> Rotation2d:
        return Rotation2d(self._inputs.turn_absolute_position)

    def setSwerveAngleTarget(self, swerve_angle_target: Rotation2d) -> None:
        steer_encoder_target = swerve_angle_target.radians() / constants.RADIANS_PER_REVOLUTION
        Logger.recordOutput(f"Drive/{self.name}/AngleTarget", steer_encoder_target)

        self._steer_motor.set_control(self.steer_request.with_position(steer_encoder_target))

    def getWheelLinearVelocity(self) -> float:
        return self._inputs.drive_velocity * self._wheel_radius

    def getWheelTotalPosition(self) -> float:
        return self._inputs.drive_position * self._wheel_radius

    def setWheelLinearVelocityTarget(self, wheel_linear_velocity_target: float) -> None:
        drive_encoder_target = wheel_linear_velocity_target / constants.RADIANS_PER_REVOLUTION
        Logger.recordOutput(f"Drive/{self.name}/DriveTarget", drive_encoder_target)

        self._drive_motor.set_control(self.drive_request.with_velocity(drive_encoder_target))

    def reset(self) -> None:
        self.setSwerveAngle(self.getSwerveEncoderAngle())

    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(self.getWheelTotalPosition(), self.getSwerveAngle())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getWheelLinearVelocity(), self.getSwerveAngle())

    def apply_states(self, state: SwerveModuleState) -> None:
        state.optimize(self.getSwerveAngle())

        self.setWheelLinearVelocityTarget(state.speed)

        # prevent unnecessary movement for what would otherwise not move the robot
        if abs(state.speed) >= self._min_wheel_linear_velocity:
            self.setSwerveAngleTarget(state.angle)

    def periodic(self) -> None:
        LogTracer.resetOuter("SwerveModule/" + self.name)
        self.prev_position = self.getPosition()
        LogTracer.record("GetPosition")

        self.updateInputs(self._inputs)
        LogTracer.record("UpdateInputs")

        Logger.processInputs(f"Drive/Module-{self.name}", self._inputs)
        LogTracer.record("ProcessInputs")
        LogTracer.recordTotal()

    def updateInputs(self, inputs: SwerveModuleIO.SwerveModuleIOInputs) -> None:
        inputs.drive_connected = StatusSignal.is_all_good(self.drive_position,
                                                          self.drive_velocity,
                                                          self.drive_applied,
                                                          self.drive_supply_current,
                                                          self.drive_torque_current)

        inputs.steer_connected = StatusSignal.is_all_good(self.turn_position,
                                                          self.turn_velocity,
                                                          self.turn_applied,
                                                          self.turn_supply_current,
                                                          self.turn_torque_current)

        inputs.encoder_connected = StatusSignal.is_all_good(self.turn_absolute_position)

        inputs.drive_position = self.drive_position.value * constants.RADIANS_PER_REVOLUTION
        inputs.drive_velocity = self.drive_velocity.value * constants.RADIANS_PER_REVOLUTION
        inputs.drive_applied = self.drive_applied.value
        inputs.drive_supply_current = self.drive_supply_current.value
        inputs.drive_torque_current = self.drive_torque_current.value

        inputs.turn_position = self.turn_position.value * constants.RADIANS_PER_REVOLUTION
        inputs.turn_velocity = self.turn_velocity.value * constants.RADIANS_PER_REVOLUTION
        inputs.turn_applied = self.turn_applied.value
        inputs.turn_supply_current = self.turn_supply_current.value
        inputs.turn_torque_current = self.turn_torque_current.value

        inputs.turn_absolute_position = self.turn_absolute_position.value * constants.RADIANS_PER_REVOLUTION
