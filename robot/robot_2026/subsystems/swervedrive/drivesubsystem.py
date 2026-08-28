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
"""
Swerve Drive Subsystem Module

This module implements the drive subsystem for a 4-wheel swerve drive robot using the CTRE Phoenix6
library. It provides comprehensive control over swerve module states, robot pose estimation with vision
support, and system identification capabilities.

The DriveSubsystem manages:
- Four swerve modules (front-left, front-right, back-left, back-right)
- Pose estimation and odometry using gyro and module encoders
- Vision-based pose corrections via Kalman filter
- Field-relative and robot-relative drive modes
- Slew rate limiting for smooth acceleration control
- SysId characterization for drive, steer, and rotation control loops
- Simulation support with field boundary constraints

"""

from dataclasses import dataclass
from typing import TYPE_CHECKING

from lib_6107.subsystems.constants import DriveConstants
from lib_6107.subsystems.drivetrain.ctre_drivesubsystem import CtreDriveSubsystem
from lib_6107.subsystems.gyro.gyro import Gyro
from robot_2026.generated.tuner_constants import TunerSwerveDrivetrain

if TYPE_CHECKING:
    from lib_6107.robotcontainer import RobotContainer

# TODO: This needs to be tested. Perform the following on a real robot, come up with parameters/constants
#
# Measuring Overshoot
# Implement a Control Loop:
#   You cannot simply apply constant power until the target angle is reached, as the robot needs time to decelerate
#   and will inevitably overshoot. A simple Proportional (P) loop is the standard starting point for FRC teams.
#   The motor power is made proportional to the difference between the target angle and the current angle.
#
#      Formula (simplified P-loop): motorPower = (targetAngle - currentAngle) * kP
#      kP is a constant you tune to get the desired performance.
#
# Log Data:
#   Use your FRC development environment (e.g., WPILib) to log the robot's current gyro angle and the target
#   angle to a file or SmartDashboard/Shuffleboard.
#
# Perform a Test Turn:
#   Command your robot to turn to a specific, significant angle (e.g., 90 degrees) using the P-loop,
#   and log the data during the process.
#
# Analyze the Data:
#   After the test, view the logged data in a graph or spreadsheet.
# Target Angle:
#   The desired final angle (e.g., 90 degrees).
#
# Peak Angle:
#   The maximum angle the robot reaches during the turn before it starts correcting back towards the target.
#
# Calculate Overshoot:
#   The difference between the peak angle and the target angle is the overshoot.
#
# Overshoot = Peak Angle - Target Angle
#
# Correcting Overshoot
#
# The primary method for reducing overshoot is tuning your control loop.
#
# Adjust kP:
#   If your robot consistently overshoots significantly, your kP value is likely too high. Lowering it
#   will make the turn slower but more accurate.
#
# Add Derivative (D) control:
#   Implementing a full PID loop can help. The derivative term (kD) dampens the system by applying a
#   counter-force based on how fast the error is changing (i.e., the robot's turn rate), which helps
#   slow the robot down as it approaches the target.
#
# Slow Down Turns:
#   As a simple fix, reducing the maximum motor power or speed used for turns will also reduce overshoot,
#   though it makes the robot slower overall.
#
# Ensure Proper Calibration:
#   Make sure the gyro is stationary during its initial calibration phase (when the robot code starts)
#   to minimize drift and baseline errors.
#

@dataclass(slots=True)
class DriveVendorConstants:
    vendor: str = "CTRE"

    # CTRE Tuner-X Constants needed for the TunerSwerveDrivetrain
    ctre_consts: TunerConstants = None
    module_consts: tuple[SwerveModuleIO, SwerveModuleIO, SwerveModuleIO, SwerveModuleIO] = None

    # TODO: Could we move the constants above to the other 'MyxxxConstants' file to keep all in one place


class DriveSubsystem(CtreDriveSubsystem):
    """
    Swerve Drive Subsystem for 4-wheel drive FRC robot.

    This subsystem manages a 4-wheel independent swerve drive system with integrated gyro,
    odometry, vision support, and SysId characterization capabilities. It supports both
    field-relative and robot-relative driving modes with joystick input filtering.
    """

    def __init__(self,
                 ctre_consts: TunerConstants,
                 modules: tuple[SwerveModuleIO, SwerveModuleIO, SwerveModuleIO, SwerveModuleIO],
                 container: RobotContainer):
        """
        Initialize the Drive Subsystem.

        Sets up all swerve modules, gyro sensor, slew rate limiters, and SysId routines.
        Configures Phoenix6 swerve drive requests with deadband settings. In simulation mode,
        starts the simulation thread for physics updates.

        :param consts: Swerve drivetrain constants from tuner
        :param modules: List of swerve module objects (4 modules in order: FL, FR, BL, BR)
        :param container: RobotContainer reference for robot state and constants
        """
        # The CTRE DriveSubsystem uses the TunerSwerveDrivetrain to manage the swerve modules
        # and drive requests. This drivetrain is generated on a per-project basis from the
        # Tuner-X tool. It is configured with the constants and modules.

        ctre_drive_subsystem = TunerSwerveDrivetrain(ctre_consts, modules)

        # Create the DriveConstants for this subsystem.

        consts = DriveConstants()

        # The gyro/IMU sensor is created based on the constants provided during the
        # tuner-x code generation. So we need to create our wrapper here.  Set the
        # update frequency to three times the robot periodic rate to ensure we get
        # all updates.
        freq_hz: hertz = 3.0 / container.robot.period

        gyro: Gyro = Gyro.create(consts.GyroType,
                                 consts.GyroReversed,
                                 update_frequency=freq_hz,
                                 inst=ctre_drive_subsystem.pigeon2)
        gyro.initialize()

        consts.gyro = gyro
        consts.VendorConstants = DriveVendorConstants(ctre_consts=ctre_consts,
                                                      module_consts=modules)

        # Complete the base class initialization with the CTRE drive subsystem, constants,
        # modules, and container
        super().__init__(ctre_drive_subsystem, consts, container)

        # Fully initialized
        self._initialized = True