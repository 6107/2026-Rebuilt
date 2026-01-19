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

from dataclasses import dataclass, field

from pykit.autolog import autolog
from wpimath.units import radians, radians_per_second, volts, amperes

"""
SwerveDriveIO provides  drive I/O to provide log information
for AdvantageScope replay and simulation.
"""


class SwerveDriveIO:
    @autolog
    @dataclass
    class DriveIOInputs:
        left_front_position_rad: radians = 0.0
        left_front_velocity_rad_per_sec: radians_per_second = 0.0
        left_front_applied_volts: volts = 0.0
        left_front_current_amps: list[amperes] = field(default_factory=list)

        right_front_position_rad: radians = 0.0
        right_front_velocity_rad_per_sec: radians_per_second = 0.0
        right_front_applied_volts: volts = 0.0
        right_front_current_amps: list[amperes] = field(default_factory=list)

        left_rear_position_rad: radians = 0.0
        left_rear_velocity_rad_per_sec: radians_per_second = 0.0
        left_rear_applied_volts: volts = 0.0
        left_rear_current_amps: list[amperes] = field(default_factory=list)

        right_rear_position_rad: radians = 0.0
        right_rear_velocity_rad_per_sec: radians_per_second = 0.0
        right_rear_applied_volts: volts = 0.0
        right_rear_current_amps: list[amperes] = field(default_factory=list)

    def updateInputs(self, inputs: DriveIOInputs) -> None:
        pass

    def setVoltage(self, left_front_volts: float, right_front_volts: float,
                   left_rear_volts: float, right_rear_volts: float) -> None:
        pass

    def setVelocity(self,
                    left_front_rad_per_sec: radians_per_second,
                    left_front_ff_volts: volts,
                    right_front_rad_per_sec: radians_per_second,
                    left_rear_rad_per_sec: radians_per_second,
                    right_rear_rad_per_sec: radians_per_second,
                    right_front_ff_volts: volts,
                    left_rear_ff_volts: volts,
                    right_rear_ff_volts: volts) -> None:
        pass

# class SwerveDriveIOSim(SwerveDriveIO):
#     def __init__(self) -> None:
#         self.sim = DifferentialDrivetrainSim.createKitbotSim(
#             driveconstants.kGearbox,
#             driveconstants.kMotorReduction,
#             driveconstants.kWheelRadius,
#         )
#
#         self.left_applied_volts = 0.0
#         self.right_applied_volts = 0.0
#         self.closedLoop = False
#
#         self.leftPID = PIDController(driveconstants.kSimKp, 0.0, driveconstants.kSimKd)
#         self.rightPID = PIDController(driveconstants.kSimKp, 0.0, driveconstants.kSimKd)
#
#         self.leftFFVolts = 0.0
#         self.rightFFVolts = 0.0
#
#     def updateInputs(self, inputs: SwerveDriveIO.DriveIOInputs) -> None:
#         if self.closedLoop:
#             self.left_applied_volts = (
#                     self.leftPID.calculate(
#                         self.sim.getLeftVelocity() / driveconstants.kWheelRadius,
#                     )
#                     + self.leftFFVolts
#             )
#             self.right_applied_volts = (
#                     self.rightPID.calculate(
#                         self.sim.getRightVelocity() / driveconstants.kWheelRadius,
#                     )
#                     + self.rightFFVolts
#             )
#
#         self.sim.setInputs(
#             clamp(self.left_applied_volts, -12.0, 12.0),
#             clamp(self.right_applied_volts, -12.0, 12.0),
#         )
#         self.sim.update(kRobotPeriod)
#
#         inputs.left_position_rad = (
#                 self.sim.getLeftPosition() / driveconstants.kWheelRadius
#         )
#         inputs.left_velocity_rad_per_sec = (
#                 self.sim.getLeftVelocity() / driveconstants.kWheelRadius
#         )
#         inputs.left_applied_volts = self.leftAppliedVolts
#         inputs.left_current_amps = [self.sim.getLeftCurrentDraw()]
#
#         inputs.right_position_rad = (
#                 self.sim.getRightPosition() / driveconstants.kWheelRadius
#         )
#         inputs.right_velocity_rad_per_sec = (
#                 self.sim.getRightVelocity() / driveconstants.kWheelRadius
#         )
#         inputs.right_applied_volts = self.rightAppliedVolts
#         inputs.right_current_amps = [self.sim.getRightCurrentDraw()]
#
#     def setVoltage(self, leftVolts: volts, rightVolts: volts) -> None:
#         self.closedLoop = False
#         self.left_applied_volts = leftVolts
#         self.right_applied_volts = rightVolts
#
#     def setVelocity(self, left_rad_per_sec: radians_per_second,
#                     right_rad_per_sec: radians_per_second,
#                     left_ff_volts: volts,
#                     right_ff_volts: volts) -> None:
#
#         self.closedLoop = True
#         self.leftFFVolts = left_ff_volts
#         self.rightFFVolts = right_ff_volts
#         self.leftPID.setSetpoint(left_rad_per_sec)
#         self.rightPID.setSetpoint(right_rad_per_sec)
