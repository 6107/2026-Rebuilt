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
#
# Constants for source in this subdirectory will go here

import math
import os
from enum import Enum, IntEnum, unique

from wpilib import RobotBase
from wpimath.geometry import Rotation2d, Translation3d
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import kilograms, lbsToKilograms, meters, meters_per_second, radians_per_second, rotationsToRadians, \
    seconds

from generated.tuner_constants import TunerConstants  # Use Tuner X constants if available
from lib_6107.constants import *

USE_PYKIT = False


class RobotModes(Enum):
    """Enum for robot modes."""
    REAL = 1
    SIMULATION = 2
    REPLAY = 3


SIM_MODE = (
    RobotModes.REPLAY if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
    else RobotModes.SIMULATION
)
ROBOT_MODE = RobotModes.REAL if RobotBase.isReal() else SIM_MODE

##################################################################
# Robot Constants
ROBOT_MASS: kilograms = lbsToKilograms(148 - 20.3)  # 32lbs * kg per pound
# CHASSIS    = Matter(geometry.Translation3d(0, 0, units.inchesToMeters(8)), ROBOT_MASS)        TODO: Figure this out
LOOP_TIME = 0.13  # seconds, 20ms + 110ms sprk max velocity lag

###############################################################################
# Device CAN bus IDs
DRIVER_CONTROLLER_PORT = 0
SHOOTER_CONTROLLER_PORT = 1

#################################################################
# Autonomous End Game Timing

AUTONOMOUS_END_TRIGGER_TIME = 10

#################################################################
# Drive subsystem related constants
#
# Maximum speed of the robot in meters per second, used to limit acceleration.

MAX_SPEED: meters_per_second = TunerConstants.speed_at_12_volts  # TODO: Measure this
MAX_ANGULAR_SPEED: radians_per_second = rotationsToRadians(0.75)  # TODO: Measure this
MAX_ANGULAR_ACCELERATION: radians_per_second = rotationsToRadians(0.75)  # Actually is radians/second^2

# Constraint for the motion profiled robot angle controller
THETA_CONTROLLER_CONSTRAINTS = TrapezoidProfileRadians.Constraints(MAX_ANGULAR_SPEED,
                                                                   MAX_ANGULAR_ACCELERATION)
# TODO: Do we need an 'Autonomous Mode' max speed, max accel, max_angular, ...

WHEEL_RADIUS: meters = TunerConstants._wheel_radius
WHEEL_DIAMETER: meters = WHEEL_RADIUS * 3
WHEEL_CIRCUMFERENCE: meters = WHEEL_DIAMETER * math.pi

# Hold time on motor brakes when disabled
WHEEL_LOCK_TIME: seconds = 3  # seconds

# Joystick Deadband
JOYSTICK_DEADBAND = 0.1
TURN_CONSTANT = 6

GYRO_REVERSED = False  # (affects field-relative driving)


#################################################################
# Other subsystem and device constants for this year's project

@unique
class DeviceID(IntEnum):
    # Drivetrain and IMU is provided already via Tuner X
    DRIVETRAIN_LEFT_FRONT_TURNING_ID = 21
    DRIVETRAIN_LEFT_FRONT_DRIVING_ID = 22
    DRIVETRAIN_LEFT_FRONT_ENCODER_ID = 31

    DRIVETRAIN_RIGHT_FRONT_TURNING_ID = 23
    DRIVETRAIN_RIGHT_FRONT_DRIVING_ID = 24
    DRIVETRAIN_RIGHT_FRONT_ENCODER_ID = 32

    DRIVETRAIN_LEFT_REAR_TURNING_ID = 25
    DRIVETRAIN_LEFT_REAR_DRIVING_ID = 26
    DRIVETRAIN_LEFT_REAR_ENCODER_ID = 33

    DRIVETRAIN_RIGHT_REAR_TURNING_ID = 27
    DRIVETRAIN_RIGHT_REAR_DRIVING_ID = 28
    DRIVETRAIN_RIGHT_REAR_ENCODER_ID = 34

    GYRO_DEVICE_ID = 35

    # Shooter
    SHOOTER_DEVICE_ID = 10

    # Intake Subsystem
    INTAKE_DEVICE_ID = 11


#################################################################################
# IP Address Assignments.  Not used in code, but kept here for recording purposes
#                          and are the 'At Home' assigned values
TEAM = "61.07"

ROBORIO_STATIC = f"10.{TEAM}.2"
ROBOT_RADIO_STATIC = f"10.{TEAM}.1"
AP_RADIO_STATIC = f"10.{TEAM}.4"

DRIVER_STATION_STATIC = f"10.{TEAM}.3"

PHOTONVISION_STATIC = f"10.{TEAM}.11"
LIMELIGHT_STATIC = f"10.{TEAM}.12"
LIMELIGHT_ALT_STATIC = f"10.{TEAM}.13"

# mDNS (DNS names are case-insensitive)

ROBORIO_MDMS = f"roboRIO-{TEAM}-frc.local"
TEAM_LAPTOP_MDMS = f"{TEAM}-frc.local"
CHIPS_LAPTOP_MDNS = ".local"

PHOTONVISION_MDMS = "photonvision.local"  # TODO: Make unique, add team #
LIMELIGHT_MDMS = "limelight.local"  # TODO: Make unique, add team #
LIMELIGHT_ALT_MDMS = "limelight-alt.local"  # TODO: Make unique, add team #

# USB
ROBORIO_USB_STATIC = "172.22.11.2"

#################################################################################
# Camera configurations

FRONT_CAMERA_INFO = {
    "Type"   : CAMERA_TYPE_PHOTONVISION,
    "Label"  : "front",
    "Name"   : "PhotonVision",
    "Pose"   : Translation3d(x=0.40, y=-0.15, z=0.5),
    "Heading": Rotation2d.fromDegrees(0.0),
    "Localizer": False
}

REAR_CAMERA_INFO = {
    "Type"   : CAMERA_TYPE_NONE,
    "Label"  : "rear",
    "Name"   : "",
    "Pose"   : Translation3d(x=0.40, y=-0.15, z=0.5),
    "Heading": Rotation2d.fromDegrees(180.0),
    "Localizer": False
}

LEFT_CAMERA_INFO = {
    "Type"   : CAMERA_TYPE_NONE,
    "Label"  : "left",
    "Name"   : "",
    "Pose"   : Translation3d(x=0.40, y=-0.15, z=0.5),
    "Heading": Rotation2d.fromDegrees(0.0),
    "Localizer": False
}

RIGHT_CAMERA_INFO = {
    "Type"   : CAMERA_TYPE_NONE,
    "Label"  : "right",
    "Name"   : "",
    "Pose"   : Translation3d(x=0.40, y=-0.15, z=0.5),
    "Heading": Rotation2d.fromDegrees(180.0),
    "Localizer": False
}

#################################################################################
# OPENTelemetry Support

# OTEL_SERVICE_NAME = os.getenv("OTEL_SERVICE_NAME", default="cyberjagzz")
# OTEL_TRACE_EXPORTER = os.getenv("OTEL_TRACE_EXPORTER", default="console,otlp")
# OTEL_METRICS_EXPORTER = os.getenv("OTEL_METRICS_EXPORTER", default="console")
# OTEL_OLTP_ENDPOINT = os.getenv("OTEL_OLTP_ENDPOINT", default="supermicro:4317")
