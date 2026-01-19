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

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.


import math

#from rev import SparkBaseConfig, ClosedLoopConfig
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians


class NeoMotorConstants:
    FREE_SPEED_RPM = 5676

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds
    MAX_SPEED_METERS_PER_SECOND = 4.8
    MAX_ANGULAR_SPEED = 2 * math.tau  # radians per second
    MAX_TOTAL_SPEED = 1.1 * math.sqrt(2) * MAX_SPEED_METERS_PER_SECOND

    DIRECTION_SLEW_RATE = 1.2  # radians per second
    MAGNITUDE_SLEW_RATE = 1.8  # percent per second (1 = 100%)
    ROTATIONAL_SLEW_RATE = 2.0  # percent per second (1 = 100%)

    # Chassis configuration
    TRACK_WIDTH = units.inchesToMeters(8.75 + 8.75)  # From YAGSL JSON offsets
    # Distance between centers of right and left wheels on robot
    WHEEL_BASE = units.inchesToMeters(8.75 + 8.75)  # From YAGSL JSON offsets

    # Distance between front and back wheels on robot
    MODULE_POSITIONS = [
        Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
        Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
        Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
        Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2),
    ]
    DRIVE_KINEMATICS = SwerveDrive4Kinematics(*MODULE_POSITIONS)

    # set it to True if you were using a ruler for zeroing and want to ignore the offsets below
    ASSUME_ZERO_OFFSET = True

    # set the above to == False, if you are using Rev zeroing tool (and you have to tinker with offsets below)
    FRONT_LEFT_ANGULAR_CHASSIS_OFFSET = -math.pi / 2
    FRONT_RIGHT_ANGULAR_CHASSIS_OFFSET = 0
    BACK_LEFT_ANGULAR_CHASSIS_OFFSET = math.pi
    BACK_RIGHT_ANGULAR_CHASSIS_OFFSET = math.pi / 2

    # SPARK MAX Parameters
    FRONT_LEFT_ANGULAR_OFFSET = 156.445
    FRONT_LEFT_DRIVE_MOTOR_INVERTED = False
    FRONT_LEFT_TURNING_MOTOR_INVERTED = False

    FRONT_RIGHT_ANGULAR_OFFSET = 30.498
    FRONT_RIGHT_DRIVE_MOTOR_INVERTED = False
    FRONT_RIGHT_TURNING_MOTOR_INVERTED = False

    REAR_LEFT_ANGULAR_OFFSET = 133.418
    REAR_LEFT_DRIVE_MOTOR_INVERTED = False
    REAR_LEFT_TURNING_MOTOR_INVERTED = False

    REAR_RIGHT_ANGULAR_OFFSET = 99.404
    REAR_RIGHT_DRIVE_MOTOR_INVERTED = False
    REAR_RIGHT_TURNING_MOTOR_INVERTED = False

    GYRO_REVERSED = False  # (affects field-relative driving)

    GYRO_TYPE_NAVX = "navx"
    GYRO_TYPE_PIGEON2 = "pigeon2"

    # GYRO_TYPE = GYRO_TYPE_NAVX        # pick only one Gyro/IMU type
    GYRO_TYPE = GYRO_TYPE_PIGEON2
#
# def getSwerveDrivingMotorConfig(driveMotorInverted: bool) -> SparkBaseConfig:
#     drivingConfig = SparkBaseConfig()
#     drivingConfig.inverted(driveMotorInverted)
#     drivingConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
#     drivingConfig.smartCurrentLimit(ModuleConstants.DRIVING_MOTOR_CURRENT_LIMIT)
#     drivingConfig.encoder.positionConversionFactor(ModuleConstants.DRIVING_ENCODER_POSITION_FACTOR)
#     drivingConfig.encoder.velocityConversionFactor(ModuleConstants.DRIVING_ENCODER_VELOCITY_FACTOR)
#     # TODO: 2026 Support needed: drivingConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder)
#     drivingConfig.closedLoop.pid(ModuleConstants.DrivingP, ModuleConstants.DrivingI, ModuleConstants.DrivingD)
#     drivingConfig.closedLoop.velocityFF(ModuleConstants.DrivingFF)
#     drivingConfig.closedLoop.outputRange(ModuleConstants.DRIVING_MIN_OUTPUT, ModuleConstants.DRIVING_MAX_OUTPUT)
#     return drivingConfig
#
#
# def getSwerveTurningMotorConfig(turnMotorInverted: bool) -> SparkBaseConfig:
#     turningConfig = SparkBaseConfig()
#     turningConfig.inverted(turnMotorInverted)
#     turningConfig.setIdleMode(SparkBaseConfig.IdleMode.kBrake)
#     turningConfig.smartCurrentLimit(ModuleConstants.TURNING_MOTOR_CURRENT_LIMIT)
#     turningConfig.absoluteEncoder.positionConversionFactor(ModuleConstants.TURNING_ENCODER_POSITION_FACTOR)
#     turningConfig.absoluteEncoder.velocityConversionFactor(ModuleConstants.TURNING_ENCODER_VELOCITY_FACTOR)
#     turningConfig.absoluteEncoder.inverted(ModuleConstants.TURNING_ENCODER_INVERTED)
#     turningConfig.closedLoop.setFeedbackSensor(ClosedLoopConfig.FeedbackSensor.kAbsoluteEncoder)
#     turningConfig.closedLoop.pid(ModuleConstants.TurningP, ModuleConstants.TurningI, ModuleConstants.TurningD)
#     turningConfig.closedLoop.velocityFF(ModuleConstants.TurningFF)
#     turningConfig.closedLoop.outputRange(ModuleConstants.TURNING_MIN_OUTPUT, ModuleConstants.TURNING_MAX_OUTPUT)
#     turningConfig.closedLoop.positionWrappingEnabled(True)
#     turningConfig.closedLoop.positionWrappingInputRange(0, ModuleConstants.TURNING_ENCODER_POSITION_FACTOR)
#     return turningConfig


class ModuleConstants:
    # WATCH OUT:
    #  - one or both of two constants below need to be flipped from True to False (by trial and error)
    #    depending on which swerve module you have (MK4i, MK4n, Rev, WCP, ThriftyBot, etc.)
    TURNING_ENCODER_INVERTED = False
    TURNING_MOTOR_INVERTED = False

    # The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    # This changes the drive speed of the module (a pinion gear with more teeth will result in a
    # robot that drives faster).
    DRIVING_MOTOR_PINION_TEETH = 14

    # Calculations required for driving motor conversion factors and feed forward
    DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.FREE_SPEED_RPM / 60
    WHEEL_DIAMETER_METERS = 0.0762
    WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * math.pi
    # 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15)
    DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS) / DRIVING_MOTOR_REDUCTION

    DRIVING_ENCODER_POSITION_FACTOR = (WHEEL_DIAMETER_METERS * math.pi) / DRIVING_MOTOR_REDUCTION  # meters
    DRIVING_ENCODER_VELOCITY_FACTOR = ((
                                                   WHEEL_DIAMETER_METERS * math.pi) / DRIVING_MOTOR_REDUCTION) / 60.0  # meters per second

    TURNING_ENCODER_POSITION_FACTOR = math.tau  # radian
    TURNING_ENCODER_VELOCITY_FACTOR = math.tau / 60.0  # radians per second

    TURNING_ENCODER_POSITION_PID_MIN_INPUT = 0  # radian
    TURNING_ENCODER_POSITION_PID_MAX_INPUT = TURNING_ENCODER_POSITION_FACTOR  # radian

    DrivingP = 0.04
    DrivingI = 0
    DrivingD = 0
    DrivingFF = 1 / DRIVE_WHEEL_FREE_SPEED_RPS
    DRIVING_MIN_OUTPUT = -1
    DRIVING_MAX_OUTPUT = 1

    TurningP = 1  # can be dialed down if you see oscillations in the turning motor
    TurningI = 0
    TurningD = 0
    TurningFF = 0
    TURNING_MIN_OUTPUT = -1
    TURNING_MAX_OUTPUT = 1

    # DrivingMotorIdleMode = SparkBase.IdleMode.kBrake
    # TurningMotorIdleMode = SparkBase.IdleMode.kBrake

    DRIVING_MOTOR_CURRENT_LIMIT = 50  # amp
    TURNING_MOTOR_CURRENT_LIMIT = 20  # amp

    MIN_DRIVING_SPEED = 0.01  # Meters per second



class AutoConstants:
    USE_SQRT_CONTROL = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    MAX_SPEED_METERS_PER_SECOND = 3
    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = math.pi
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = math.pi

    PX_CONTROLLER = 1
    PY_CONTROLLER = 1
    P_THETA_CONTROLLER = 0.67

    # Constraint for the motion profiled robot angle controller
    THETA_CONTROLLER_CONSTRAINTS = TrapezoidProfileRadians.Constraints(MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                                                                       MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED)
