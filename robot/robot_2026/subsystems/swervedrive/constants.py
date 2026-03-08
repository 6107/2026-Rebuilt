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

# from rev import SparkBaseConfig, ClosedLoopConfig
from wpimath import units
from wpimath.geometry import Translation2d
from wpimath.kinematics import SwerveDrive4Kinematics


class NeoMotorConstants:
    FREE_SPEED_RPM = 5676

class DriveConstants:
    # Driving Parameters - Note that these are not the maximum capable speeds of
    # the robot, rather the allowed maximum speeds.
    #
    # For slew rate, this controls joystick input so that a quick swing in one
    # direction does not happen immediately. Values are units per second. For a
    # joystick the voltage values are -1.0..1.0. So if rate was 2.0, it would take
    # a full second to swing from full reverse to full forward.

    MAGNITUDE_SLEW_RATE = 4.0  # percent per second (1 = 100%)
    ROTATIONAL_SLEW_RATE = 4.0  # percent per second (1 = 100%)

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

class AutoConstants:
    USE_SQRT_CONTROL = True  # improves arrival time and precision for simple driving commands

    # below are really trajectory constants
    #MAX_SPEED_METERS_PER_SECOND = 3
    MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = math.pi
    MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = math.pi

    PX_CONTROLLER = 1
    PY_CONTROLLER = 1
    P_THETA_CONTROLLER = 0.67

