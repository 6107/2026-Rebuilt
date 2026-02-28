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
from wpimath.geometry import Rotation3d, Transform3d, Translation2d, Translation3d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import degreesToRadians, hertz, inchesToMeters, kilograms, lbsToKilograms, meters, meters_per_second, \
    radians, radians_per_second, rotationsToRadians, seconds

from lib_6107.constants import *
from robot_2026.generated.tuner_constants import TunerConstants  # Use Tuner X constants if available


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

# Robot size (including bumpers). Defaults, use PathPlanner to set actual value to
#                                 use if PathPlanner is supported
ROBOT_X_WIDTH_DEFAULT: meters = 0.4
ROBOT_Y_WIDTH_DEFAULT: meters = 0.4

###############################################################################
# Device CAN bus IDs
DRIVER_CONTROLLER_PORT = 0
SHOOTER_CONTROLLER_PORT = 1
CALIBRATION_CONTROLLER_PORT = 2  # Set to < 0 to disable initialization

DEFAULT_FREQUENCY: hertz = 50.0
ODOMETRY_FREQUENCY: hertz = 100.0  # Primarily for yaw

#################################################################
# Autonomous End Game Timing

AUTONOMOUS_END_TRIGGER_TIME = 10

#################################################################
# Drive subsystem related constants
#
# Maximum speed of the robot in meters per second, used to limit acceleration. The
# Minimum speed is used to keep the robot from moving at a very low rate

MAX_SPEED: meters_per_second = TunerConstants.speed_at_12_volts  # TODO: Measure this
MIN_SPEED: meters_per_second = 0.002
MAX_ANGULAR_SPEED: radians_per_second = rotationsToRadians(0.75)  # TODO: Measure this
MAX_ANGULAR_ACCELERATION: radians_per_second = rotationsToRadians(0.75)  # Actually is radians/second^2

# Constraint for the motion profiled robot angle controller
THETA_CONTROLLER_CONSTRAINTS = TrapezoidProfileRadians.Constraints(MAX_ANGULAR_SPEED,
                                                                   MAX_ANGULAR_ACCELERATION)
# TODO: Do we need an 'Autonomous Mode' max speed, max accel, max_angular, ...

WHEEL_RADIUS: meters = TunerConstants._wheel_radius
WHEEL_DIAMETER: meters = WHEEL_RADIUS * 3
WHEEL_CIRCUMFERENCE: meters = WHEEL_DIAMETER * math.pi

MAX_WHEEL_LINEAR_VELOCITY: meters_per_second = 1.0

# Hold time on motor brakes when disabled
WHEEL_LOCK_TIME: seconds = 3  # seconds

# Joystick Deadband
JOYSTICK_DEADBAND = 0.1

GYRO_REVERSED = False  # (affects field-relative driving)

DriveKinematics = SwerveDrive4Kinematics(
    Translation2d(TunerConstants._front_left_x_pos, TunerConstants._front_left_y_pos),
    Translation2d(TunerConstants._front_right_x_pos, TunerConstants._front_right_y_pos),
    Translation2d(TunerConstants._back_left_x_pos, TunerConstants._back_left_y_pos),
    Translation2d(TunerConstants._back_right_x_pos, TunerConstants._back_right_y_pos)
)

#################################################################
# Other subsystem and device constants for this year's project

@unique
class DeviceID(IntEnum):
    # Drivetrain and IMU is provided already via Tuner X
    DRIVETRAIN_LEFT_FRONT_TURNING_ID = TunerConstants._front_left_steer_motor_id
    DRIVETRAIN_LEFT_FRONT_DRIVING_ID = TunerConstants._front_left_drive_motor_id
    DRIVETRAIN_LEFT_FRONT_ENCODER_ID = TunerConstants._front_left_encoder_id

    DRIVETRAIN_RIGHT_FRONT_TURNING_ID = TunerConstants._front_right_steer_motor_id
    DRIVETRAIN_RIGHT_FRONT_DRIVING_ID = TunerConstants._front_right_drive_motor_id
    DRIVETRAIN_RIGHT_FRONT_ENCODER_ID = TunerConstants._front_right_encoder_id

    DRIVETRAIN_LEFT_REAR_TURNING_ID = TunerConstants._back_left_steer_motor_id
    DRIVETRAIN_LEFT_REAR_DRIVING_ID = TunerConstants._back_left_drive_motor_id
    DRIVETRAIN_LEFT_REAR_ENCODER_ID = TunerConstants._back_left_encoder_id

    DRIVETRAIN_RIGHT_REAR_TURNING_ID = TunerConstants._back_right_steer_motor_id
    DRIVETRAIN_RIGHT_REAR_DRIVING_ID = TunerConstants._back_right_drive_motor_id
    DRIVETRAIN_RIGHT_REAR_ENCODER_ID = TunerConstants._back_right_encoder_id

    GYRO_DEVICE_ID = TunerConstants._pigeon_id

    # Shooter
    SHOOTER_DEVICE_ID = 30

    # Intake Subsystem
    INTAKE_DEVICE_ID = 31

    # Climber Subsystem
    CLIMBER_DEVICE_ID = 33

#################################################################################
# IP Address Assignments.  Not used in code, but kept here for recording purposes
#                          and are the 'At Home' assigned values
TEAM = "61.07"

ROBORIO_STATIC = f"10.{TEAM}.2"
ROBOT_RADIO_STATIC = f"10.{TEAM}.1"
AP_RADIO_STATIC = f"10.{TEAM}.4"

DRIVER_STATION_STATIC = f"10.{TEAM}.5"
DRIVER_STATION_ALT_STATIC = f"10.{TEAM}.6"

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
    "Type"     : CAMERA_TYPE_PHOTONVISION,
    "Label"    : "front",
    "Name"     : "PhotonVision",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(-5.0),
                                           y=inchesToMeters(0.0),
                                           z=inchesToMeters(12.0)),
                             Rotation3d(0.0, 0.0, degreesToRadians(0.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

REAR_CAMERA_INFO = {
    "Type": CAMERA_TYPE_NONE,  # TODO: CAMERA_TYPE_LIMELIGHT,
    "Label"    : "rear",
    "Name"     : "LimeLight",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(-6.0),
                                           y=inchesToMeters(0.0),
                                           z=inchesToMeters(12.0)),
                             Rotation3d(0.0, 0.0, degreesToRadians(180.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

LEFT_CAMERA_INFO = {
    "Type"     : CAMERA_TYPE_NONE,
    "Label"    : "left",
    "Name"     : "",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(0),
                                           y=inchesToMeters(0),
                                           z=inchesToMeters(0)),
                             Rotation3d(0.0, 0.0, degreesToRadians(90.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

RIGHT_CAMERA_INFO = {
    "Type"     : CAMERA_TYPE_NONE,
    "Label"    : "right",
    "Name"     : "",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(0),
                                           y=inchesToMeters(0),
                                           z=inchesToMeters(0)),
                             Rotation3d(0.0, 0.0, degreesToRadians(270.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

# Vision Pose filter limits
# TODO: Look into the april tag heights and how Z_ERROR is actually used and obtained and
#       see if we can calculate it from the field data on startup to be 1/2 meter above the
#       maximum
MAX_VISION_AMBIGUITY = 0.3
MAX_VISION_Z_ERROR: meters = 1.5  # 0.75

# Adjusted automatically based on distance and # of tags
LINEAR_STD_DEV_BASELINE: meters = 0.02
ANGULAR_STD_DEV_BASELINE: radians = 0.06

# Multipliers to apply for MegaTag 2 observations
LINEAR_STD_DEV_MEGATAG2_FACTOR: float = 0.5  # More stable than full 3D solve
ANGULAR_STD_DEV_MEGATAG2_FACTOR: float = math.inf  # No rotation data available

# Vision Pipeline for Apriltags. The pipelines need to be manually
# set up in the camera and should use the following pipeline number. (0..9).
APRILTAGS_PIPELINE = 0  # TODO: Need to set this up in all our cameras

#################################################################################
# OPENTelemetry Support

# OTEL_SERVICE_NAME = os.getenv("OTEL_SERVICE_NAME", default="cyberjagzz")
# OTEL_TRACE_EXPORTER = os.getenv("OTEL_TRACE_EXPORTER", default="console,otlp")
# OTEL_METRICS_EXPORTER = os.getenv("OTEL_METRICS_EXPORTER", default="console")
# OTEL_OLTP_ENDPOINT = os.getenv("OTEL_OLTP_ENDPOINT", default="supermicro:4317")
