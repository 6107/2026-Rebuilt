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
import logging
import math
from typing import Optional

from phoenix6 import BaseStatusSignal, StatusCode
from phoenix6.configs import Pigeon2Configuration
from phoenix6.hardware import pigeon2
from phoenix6.sim.pigeon2_sim_state import Pigeon2SimState
from wpilib import SmartDashboard, RobotBase
from wpimath.geometry import Rotation2d
from wpimath.units import degrees, degrees_per_second, hertz

from lib_6107.constants import RADIANS_PER_DEGREE
from lib_6107.subsystems.gyro.gyro import Gyro, GyroIO

logger = logging.getLogger(__name__)


class Pigeon2(Gyro):
    """
    Pigeon2 gyro implementation
    """
    gyro_type = "Pigeon2"

    def __init__(self, device_id: int, is_reversed: bool, update_frequency: hertz, inst: Optional[Pigeon2] = None) -> None:
        if inst is not None:
            # Supplied by operator. For Pigeon2, use the Pigeon 2 calibration tool in the CTRE Tuner X
            # to set the orientation.
            assert isinstance(inst, pigeon2.Pigeon2), f"Invalid object type past in as gyro instance: {type(inst)}"
            is_reversed = False

        # Initialize base class
        super().__init__(is_reversed)

        self._gyro: pigeon2.Pigeon2 = inst or pigeon2.Pigeon2(device_id)
        self._sim_gyro: Optional[pigeon2.Pigeon2] = None
        self._sim_gyro_state: Optional[Pigeon2SimState] = None
        self._instance_supplied = inst is not None

        # Note: Default pigeon2 config has compass disabled. We want it that way as well.
        config: Pigeon2Configuration = Pigeon2Configuration()
        config.pigeon2_features.enable_compass = False

        self._gyro.configurator.apply(config)
        self._update_hz = update_frequency

        # Next two are for use by pykit for AdvantageScope support
        self._yaw: BaseStatusSignal = self._gyro.get_yaw()
        self._yaw_velocity = self._gyro.get_angular_velocity_z_world()

    def initialize(self) -> None:
        """
        Perform initial steps to get your gyro ready
        """
        if not self._instance_supplied:
            # Only initialize if this class did the initial initialization of the Pigeon2 object

            self.reset()
            if self._update_hz > 0.0:
                # TODO: See about maybe updating faster than the base frequency. Perhaps twice as
                #       fast from the update_hz value. This may provide a little better results if
                #       the robot periodic call skews with respect to the Pigeon2 updates.

                status = BaseStatusSignal.set_update_frequency_for_all(self._update_hz,
                                                                       self._yaw,
                                                                       self._yaw_velocity)
                if status != StatusCode.OK:
                    logger.warning(f"{self.gyro_type}: Error during gyro frequency update: {status}")

            status = self._gyro.optimize_bus_utilization()

            if status != StatusCode.OK:
                logger.warning(f"{self.gyro_type}: Error during gyro bus optimization: {status}")

    @property
    def calibrated(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return True

    @property
    def is_calibrating(self) -> bool:
        """
        Is this gyro calibrated. Implement in derived class if your gyro
        does not auto-calibrate.
        """
        return False

    def reset(self) -> None:
        """
        Reset the gyro
        """
        self.zero_yaw()

    def zero_yaw(self) -> None:
        self._gyro.set_yaw(0.0)  # we boot up at zero degrees  - note - you can't reset this while calibrating

        if RobotBase.isSimulation():
            print("TODO: Need to zero simulation gyro")  # TODO

    @property
    def yaw(self) -> degrees:
        """
        TODO: Look into claim below and validate it. Then see where we use this. Navx may be the
              only one with 'angle'
        helpful for determining nearest heading parallel to the wall,
        but you should probably never use this - just use get_angle to be consistent
        because yaw does NOT return the offset that get_Angle does
        """
        yaw = self._gyro.get_yaw().value

        return -yaw if self._reversed else yaw

    @property
    def pitch(self) -> degrees:
        pitch_offset = 0  # TODO: Always zero?

        return self._gyro.get_pitch().value - pitch_offset

    @property
    def roll(self) -> degrees:
        roll_offset = 0  # TODO: Always zero?

        return self._gyro.get_roll().value - roll_offset

    @property
    def raw_angle(self) -> degrees:
        return self.angle

    @property
    def angle(self) -> degrees:
        return self.yaw

    @property
    def turn_rate(self) -> float:
        """
        Returns the turn rate of the robot (in radians per second)
        """
        return math.radians(self.turn_rate_degrees_per_second)

    @property
    def turn_rate_degrees_per_second(self) -> degrees_per_second:
        rate = self._gyro.get_angular_velocity_z_world().value

        return -rate if self._reversed else rate

    ######################
    # pykit / AdvantageScope support

    def updateInputs(self, inputs: GyroIO.GyroIOInputs) -> None:
        inputs.connected = BaseStatusSignal.refresh_all(self._yaw,
                                                        self._yaw_velocity).is_ok()
        inputs.yawPosition = Rotation2d.fromDegrees(self._yaw.value_as_double)
        inputs.yawVelocityDegPerSec = (self._yaw_velocity.value_as_double *
                                       RADIANS_PER_DEGREE)

        self._yaw: BaseStatusSignal = self._gyro.get_yaw()
        self._yaw_velocity = self._gyro.get_angular_velocity_z_world()

    ######################
    # SmartDashboard support

    def dashboard_initialize(self) -> None:
        SmartDashboard.putString('Gyro/type', self.gyro_type)

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        super().dashboard_periodic()

        # Pigeon has an all-good static to test if all is okay with the world
        SmartDashboard.putBoolean('Gyro/all-good', BaseStatusSignal.is_all_good())

    ######################
    # Simulation support

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Create your simulation gyro (and set  self._sim_gyro)
        """
        super().sim_init(physics_controller)

        self._sim_gyro = self._gyro
        self._sim_gyro_state = self._gyro.sim_state

    @property
    def sim_yaw(self) -> degrees:
        return self._sim_gyro.get_yaw().value

    @sim_yaw.setter
    def sim_yaw(self, value: degrees) -> None:
        """
        Used during simulation
        """
        # TODO: Copied from physics.py reconcile with any existing functions in this class
        if self._reversed:
            value = -value

        self._sim_gyro_state.set_raw_yaw(value)
