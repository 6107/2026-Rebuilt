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

from dataclasses import dataclass
from enum import Enum, IntEnum, unique

from wpilib import RobotBase
from wpimath.geometry import Rotation3d, Transform3d, Translation2d, Translation3d
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.trajectory import TrapezoidProfileRadians
from wpimath.units import degreesToRadians, hertz, inchesToMeters, kilograms, lbsToKilograms, meters, meters_per_second, \
    radians, radians_per_second, rotationsToRadians, seconds

from lib_6107.constants import RobotConstants, CameraType, NetworkConstants
from robot_2026.generated.tuner_constants import TunerConstants  # Use Tuner X constants if available


@dataclass(slots=True)
class MyRobotConstants(RobotConstants):
    """
    Update for fields unique to the 2026-Rebuilt robot
    """
    ROBOT_MASS: kilograms = lbsToKilograms(60)

    ROBOT_CHASSIS_X_WIDTH: meters = inchesToMeters(27)
    ROBOT_CHASSIS_Y_WIDTH: meters = inchesToMeters(27)

    ###########################
    # Robot periodic rate
    ROBOT_PERIOD: seconds = 0.0333

    ###########################
    # Drivetrain  (often overridden in real-robot based on Tuner-X constants)
    MAX_SPEED: meters_per_second = TunerConstants.speed_at_12_volts
    MAX_ANGULAR_VELOCITY: radians_per_second = rotationsToRadians(0.75)  # TODO: Measure this

#################################################################
# Autonomous End Game Timing

AUTONOMOUS_END_TRIGGER_TIME = 10

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
    REV_POWER_DISTRIBUTION_HUB = 1

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

    # Intake Subsystem
    INTAKE_INDEXER_DEVICE_ID = 30
    INTAKE_LEFT_PIVOT_DEVICE_ID = 31
    INTAKE_RIGHT_PIVOT_DEVICE_ID = 32
    INTAKE_ROLLER_DEVICE_ID = 33

    # Shooter
    SHOOTER_DEVICE_ID = 34

    # Climber Subsystem
    CLIMBER_DEVICE_ID = 35

#################################################################################
# IP Address Assignments.  Not used in code, but kept here for recording purposes
#                          and are the 'At Home' assigned values
@dataclass(slots=True)
class MyNetworkConstants(NetworkConstants):

    RADIO_2_MAC_ADDRESS = "48:DA:35:B0:B1:E0"  # Not used, but helps us tell them apart since we have two available
    RADIO_2_WIFI_PWD = "NameNumber"

    # Following are for recording purposes only. Also not used..
    # PHOTONVISION_STATIC = f"10.{super().TEAM}.11"
    # LIMELIGHT_STATIC = f"10.{super().TEAM}.12"
    # LIMELIGHT_ALT_STATIC = f"10.{super().TEAM}.13"
    #
    # # mDNS (DNS names are case-insensitive)
    # TEAM_LAPTOP_MDMS = f"{super().TEAM}-frc.local"
    #
    # PHOTON_VISION_NAME = "pf-6107-frc"
    # PHOTONVISION_MDMS = f"{PHOTON_VISION_NAME}.local"
    # LIMELIGHT_MDMS = "limelight.local"  # TODO: Make unique, add team #
    # LIMELIGHT_ALT_MDMS = "limelight-alt.local"  # TODO: Make unique, add team #


#################################################################################
# Camera configurations     TODO: Move this to subsystems

FRONT_CAMERA_INFO = {
    "Type"     : CameraType.NONE,   # CAMERA_TYPE_PHOTONVISION,
    "Label"    : "front",
    "Name"     : "PhotonVision",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(-11.0),
                                           y=inchesToMeters(4.25),
                                           z=inchesToMeters(13.625)),
                             Rotation3d(degreesToRadians(0.0),  # Roll  -  front/back x-axis rotation
                                        degreesToRadians(-3.0),  # Pitch -  side/side y-axis rotation
                                        degreesToRadians(0.0))),# Yaw   - left(+) / right(-) z-axis
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

# TODO: Transform into a re-usable class
REAR_CAMERA_INFO = {
    "Type"     : CameraType.NONE, # CameraTypes.CAMERA_TYPE_LIMELIGHT,
    "Label"    : "rear",
    "Name"     : "LimeLight",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(2.5),
                                           y=inchesToMeters(-10.5),
                                           z=inchesToMeters(31.625)),
                             Rotation3d(0.0, 0.0, degreesToRadians(180.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

LEFT_CAMERA_INFO = {
    "Type"     : CameraType.NONE,
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
    "Type"     : CameraType.NONE,
    "Label"    : "right",
    "Name"     : "",
    "Transform": Transform3d(Translation3d(x=inchesToMeters(0),
                                           y=inchesToMeters(0),
                                           z=inchesToMeters(0)),
                             Rotation3d(0.0, 0.0, degreesToRadians(270.0))),
    "Localizer": False,
    "Trust"    : 1.0  # [0.0..1.0] More trusted cameras are closer to 1.0
}

#################################################################################
# OPENTelemetry Support

# OTEL_SERVICE_NAME = os.getenv("OTEL_SERVICE_NAME", default="cyberjagzz")
# OTEL_TRACE_EXPORTER = os.getenv("OTEL_TRACE_EXPORTER", default="console,otlp")
# OTEL_METRICS_EXPORTER = os.getenv("OTEL_METRICS_EXPORTER", default="console")
# OTEL_OLTP_ENDPOINT = os.getenv("OTEL_OLTP_ENDPOINT", default="supermicro:4317")
