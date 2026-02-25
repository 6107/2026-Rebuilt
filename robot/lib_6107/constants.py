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
# Commonly used constants not found in existing wpilib modules

from math import pi

# The period is available from robot.GetPeriod() and the following provides
# a default value in case it returns 0 or None
DEFAULT_ROBOT_FREQUENCY = 1.0 / 50

######################################################################
# Subsystem related constants

CAMERA_TYPE_NONE = ""
CAMERA_TYPE_LIMELIGHT = "Limelight"  # Currently Limelight 2 only
CAMERA_TYPE_PHOTONVISION = "PhotonVision"


######################################################################
# Math
RADIANS_PER_REVOLUTION = 2 * pi
DEGREES_PER_REVOLUTION = 360.0
RADIANS_PER_DEGREE = RADIANS_PER_REVOLUTION / DEGREES_PER_REVOLUTION

INCHES_PER_FOOT = 12.0
CENTIMETERS_PER_METER = 100.0
CENTIMETERS_PER_INCH = 2.54
METERS_PER_INCH = CENTIMETERS_PER_INCH / CENTIMETERS_PER_METER

SECONDS_PER_MINUTE = 60.0
MINUTES_PER_HOUR = 60.0
