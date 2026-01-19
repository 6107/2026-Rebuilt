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
import os
from enum import Enum

from wpilib import RobotBase
from wpimath import units

USE_PYKIT = False

class RobotModes(Enum):
    """Enum for robot modes."""

    REAL = 1
    SIMULATION = 2
    REPLAY = 3


kSimMode = (
    RobotModes.REPLAY
    if "LOG_PATH" in os.environ and os.environ["LOG_PATH"] != ""
    else RobotModes.SIMULATION
)
kRobotMode = RobotModes.REAL if RobotBase.isReal() else kSimMode

###############################################################################
# OPENTelemetry Support
OTEL_SERVICE_NAME = os.getenv("OTEL_SERVICE_NAME", default="cyberjagzz")
OTEL_TRACE_EXPORTER = os.getenv("OTEL_TRACE_EXPORTER", default="console,otlp")
OTEL_METRICS_EXPORTER = os.getenv("OTEL_METRICS_EXPORTER", default="console")
OTEL_OLTP_ENDPOINT = os.getenv("OTEL_OLTP_ENDPOINT", default="supermicro:4317")

# Constants from the 2025 JAVA program for the CyberJagzz robot
#
# The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
# class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
# not put anything functional in this class.

ROBOT_MASS = units.lbsToKilograms(148 - 20.3)  # 32lbs * kg per pound
# CHASSIS    = Matter(geometry.Translation3d(0, 0, units.inchesToMeters(8)), ROBOT_MASS)        TODO: Figure this out
LOOP_TIME = 0.13  # seconds, 20ms + 110ms sprk max velocity lag

# Maximum speed of the robot in meters per second, used to limit acceleration.
#    1.45 feet is 4.4196 meters
MAX_SPEED = units.feetToMeters(14.5)

I_INTAKE_EXTEND_MAX = 100

# Hold time on motor brakes when disabled
WHEEL_LOCK_TIME = 3  # seconds

# Joystick Deadband
JOYSTICK_DEADBAND = 0.1
TURN_CONSTANT = 6

DRIVER_CONTROLLER_PORT = 0
SHOOTER_CONTROLLER_PORT = 1
