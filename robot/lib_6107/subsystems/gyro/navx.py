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

import math
from typing import Any, Optional

from wpilib import RobotBase
from wpilib.simulation import SimDeviceSim
from wpimath.geometry import Rotation2d
from wpimath.units import degrees, degrees_per_second, radians_per_second

try:
    import navx

    NAVX_SUPPORTED = True
except ImportError:
    NAVX_SUPPORTED = False

from lib_6107.constants import RADIANS_PER_DEGREE
from lib_6107.subsystems.gyro.gyro import Gyro, GyroIO


class NavX(Gyro):
    """
    NavX gyro implementation
    """
    gyro_type = "navX"

    def __init__(self, is_reversed: bool, inst: Optional[Any] = None):
        super().__init__(is_reversed)

        self._gyro = inst or navx.AHRS.create_spi()
        self._sim_gyro: Optional[SimDeviceSim] = None
        self._is_simulation = RobotBase.isSimulation()
        self._calibrated = False

    def initialize(self) -> None:
        """
        Perform initial steps to get your gyro ready
        """
        if self.is_calibrating:
            # Flag that gyro is not calibrated. Checked in periodic call
            self._calibrated = False
        else:
            self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
            self._calibrated = True

    @property
    def calibrated(self) -> bool:
        """
        Is this gyro calibrated.
        """
        return self._calibrated

    @property
    def is_calibrating(self) -> bool:
        """
        Is this gyro calibrated.
        """
        return self._gyro.isCalibrating() or not self._is_simulation

    def reset(self, adjustment=None) -> None:
        """
        Reset the gyro
        """
        if self._gyro.zeroYaw():
            self.zero_yaw()

        else:
            self._gyro.reset()

    def zero_yaw(self) -> None:
        if self._is_simulation:
            self.sim_yaw = 0.0

        else:
            self._gyro.zeroYaw()

    @property
    def yaw(self) -> degrees:
        """
        helpful for determining nearest heading parallel to the wall,
        but you should probably never use this - just use get_angle to be consistent
        because yaw does NOT return the offset that get_Angle does
        """
        yaw = self._gyro.getYaw() if not self._is_simulation else self.sim_yaw

        return -yaw if self._reversed else yaw

    @property
    def pitch(self) -> degrees:
        pitch_offset = 0  # TODO: Always zero?

        return self._gyro.getPitch() - pitch_offset if not self._is_simulation else 0.0

    @property
    def roll(self) -> degrees:
        roll_offset = 0  # TODO: Always zero?

        return self._gyro.getRoll() - roll_offset if not self._is_simulation else 0.0

    @property
    def raw_angle(self) -> degrees:
        return self._gyro.getYaw() if not self._is_simulation else 0.0

    @property
    def angle(self) -> degrees:
        angle = self._gyro.getAngle() if not self._is_simulation else self.sim_yaw

        return -angle if self._reversed else angle

    @property
    def turn_rate(self) -> radians_per_second:
        """Returns the turn rate of the robot (in radians per second)

        :returns: The turn rate of the robot, in radians per second
        """
        return math.radians(self.turn_rate_degrees_per_second)

    @property
    def turn_rate_degrees_per_second(self) -> degrees_per_second:
        rate = self._gyro.getRate()

        return -rate if self._reversed else rate

    def periodic(self) -> None:
        """
        Perform any periodic maintenance
        """
        if not self.calibrated and not self.is_calibrating:
            # Gyro has finished calibrating, set it to zero
            self.zero_yaw()  # we boot up at zero degrees  - note - you can't reset this while calibrating
            self._calibrated = True

    ######################
    # pykit / AdvantageScope support

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = self._gyro.isConnected()
        inputs.yawPosition = Rotation2d.fromDegrees(self.angle)

        gyro_z: degrees_per_second = self._gyro.getRawGyroZ()
        if self.is_reversed:
            gyro_z = -gyro_z

        inputs.yawVelocityDegPerSec = gyro_z * RADIANS_PER_DEGREE

        # TODO: If timestamps are desired, look at AdvantageKit's Java template
        #       for the talonfx_swerve

    ######################
    # Simulation support

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Create your simulation gyro (and set  self._sim_gyro)
        """
        super().sim_init(physics_controller)

        # NavX (SPI interface)
        self._sim_gyro: SimDeviceSim = SimDeviceSim("navX-Sensor[4]")

    @property
    def sim_yaw(self) -> degrees:
        return self._sim_gyro.getDouble("Yaw").get()

    @sim_yaw.setter
    def sim_yaw(self, value: degrees) -> None:
        """
        Used during simulation
        """
        # TODO: Copied from physics.py reconcile with any existing functions in this class
        if self._reversed:
            value = -value

        self._sim_gyro.setDouble("yaw", value)
