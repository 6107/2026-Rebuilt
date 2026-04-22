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

import constants
import math
from commands2 import Subsystem
from robot_2026.subsystems.ctre_pivot import PivotConstants
from robot_2026.util.logtracer import LogTracer
from wpilib import Color, Color8Bit, DriverStation, Mechanism2d, RobotBase, SmartDashboard
from wpimath.kinematics import ChassisSpeeds
from wpimath.units import degrees, degrees_per_second, inchesToMeters, meters, meters_per_second, \
    revolutions_per_minute


class RobotMech(Subsystem):
    """
    This class handles all the mechanism simulation for the robot so that the entire side-view
    of the robot can be visualized.  Also helps keep some clutter out of the other subsystem
    code
    """

    def __init__(self, container: 'RobotContainer') -> None:
        """
        This subsystem should be created last so that the periodic method is called last after
        all other subsystems have updated
        """
        Subsystem.__init__(self)

        self._container = container
        self._robot = container.robot
        self._physics_controller = None

        # Approximate robot width (y-dimension) with the intake deployed and the height (z-dimension)
        # with the intake/climber/ and other subsystems at max h
        self._width = inchesToMeters(constants.ROBOT_X_WIDTH_DEFAULT + 6)
        self._height = inchesToMeters(30)

        # Robot starting X offset (so it is not starting on left/bottom side of display box)
        self._start_x = inchesToMeters(5)

        self._mech = Mechanism2d(self._width + (2 * self._start_x), self._height + (2 * self._start_x))

        # ----------------- Visual Constants -----------------
        self._color_chassis = Color8Bit(Color.kYellow)
        self._color_wheel = Color8Bit(Color.kWhite)
        self._color_intake = Color8Bit(Color.kOrange)
        self._color_hopper = Color8Bit(Color.kPurple)
        self._color_indexer = Color8Bit(Color.kCyan)
        self._color_shooter = Color8Bit(Color.kRed)
        self._color_climber = Color8Bit(Color.kGreen)
        self._color_fuel = Color8Bit(Color.kYellow)

        self._line_weight = 10  # 1 inch approx 10 pixels?

        # ----------------- Initialization -----------------
        self._init_drivetrain()
        self._init_intake()
        self._init_hopper()
        self._init_indexer()
        self._init_shooter()
        self._init_climber()
        self._init_fuel()

        # Put to dashboard
        # mech_prefix = constants.mech_prefix
        SmartDashboard.putData("Mech Side View", self._mech)

    def _get_rel_angle(self, target_abs: degrees, parent_abs: degrees) -> degrees:
        """
        Calculates the relative angle needed for a ligament given the
        desired absolute angle and the parent's absolute angle.
        """
        return target_abs - parent_abs

    def _adjust_intake_angle(self, angle: degrees) -> degrees:
        """
        Since we start with the intake up and the encoder considers that 0 degrees
        and not 90, then we need to adjust
        """
        return 90.0 - angle

    def _init_drivetrain(self):
        # Chassis: 2" bar, 27" long. Top at 4.5".
        # Center of bar is at 3.5" (since it's 2" thick, 2.5" to 4.5")               # TODO: Verify location/size
        chassis_ligamentroot_chassis = self._mech.getRoot("chassis_root", self._start_x, inchesToMeters(3.5))
        self.chassis_ligament = chassis_ligamentroot_chassis.appendLigament("chassis",
                                                                            constants.ROBOT_CHASSIS_WIDTH,
                                                                            0,
                                                                            20,
                                                                            self._color_chassis)
        # Wheels: 4" diameter (2" radius). Center at Y=2.
        # 5" in from edges (0 and 27). So at 5" and 22".               # TODO: Verify location/size
        # Rear Wheel
        self.root_rear_wheel = self._mech.getRoot("rear_wheel",
                                                  self._start_x + inchesToMeters(5),
                                                  constants.WHEEL_RADIUS)

        self.rear_wheel_ligament = self.root_rear_wheel.appendLigament("rear_spoke",
                                                                       constants.WHEEL_RADIUS,
                                                                       0,
                                                                       6,
                                                                       self._color_wheel)

        self.rear_wheel_ligament_2 = self.root_rear_wheel.appendLigament("rear_spoke_2",
                                                                         constants.WHEEL_RADIUS,
                                                                         180,
                                                                         6,
                                                                         self._color_wheel)
        # Front Wheel
        self.root_front_wheel = self._mech.getRoot("front_wheel",
                                                   self._start_x + constants.ROBOT_CHASSIS_WIDTH - inchesToMeters(5),
                                                   constants.WHEEL_RADIUS)

        self.front_wheel_ligament = self.root_front_wheel.appendLigament("front_spoke",
                                                                         constants.WHEEL_RADIUS,
                                                                         0,
                                                                         6,
                                                                         self._color_wheel)

        self.front_wheel_ligament_2 = self.root_front_wheel.appendLigament("front_spoke_2",
                                                                           constants.WHEEL_RADIUS,
                                                                           180,
                                                                           6,
                                                                           self._color_wheel)

    def _init_intake(self):
        """
        Intake pivot and rollers.
        """
        # Mounting is the lexan bracket that the intake pivot attaches to. Simulate
        # it as a post: Sticks up 4" from top of drivetrain (Y=4.5).                # TODO: Verify location/size
        # We place it at the front of the chassis (X = start + 27).
        self.root_intake_mount = self._mech.getRoot("intake_mount",
                                                    self._start_x + constants.ROBOT_CHASSIS_WIDTH,
                                                    inchesToMeters(4.5))

        self._abs_intake_mount = 0  # Roots are 0

        # The vertical mount post (the lexan)
        self._abs_intake_post_angle = 90
        self.intake_post = self.root_intake_mount.appendLigament("intake_post",
                                                                 inchesToMeters(4),
                                                                 self._get_rel_angle(self._abs_intake_post_angle,
                                                                                     self._abs_intake_mount),
                                                                 6,
                                                                 self._color_intake)
        # The pivoting arm. Pivot is at top of post (Y=8.5).
        # Needs to reach ground (Y=0). Length approx 11-12".\
        # However. Since we start with the intake up and the encoder considers that 0 degrees
        #          and not 90, then we need to adjust
        self._abs_intake_arm_start_angle = self._adjust_intake_angle(PivotConstants.RETRACTED_ANGLE)  # Initial angle
        self.intake_arm_length = inchesToMeters(10)
        self.intake_arm = self.intake_post.appendLigament("intake_arm",
                                                          self.intake_arm_length,
                                                          self._get_rel_angle(self._abs_intake_arm_start_angle,
                                                                              self._abs_intake_post_angle),
                                                          6,
                                                          self._color_intake)
        # Animation: Spacer and Indicator (something to animate to show that it is on)
        self.intake_anim_dist = 0
        self.intake_spacer = self.intake_arm.appendLigament("intake_spacer",
                                                            inchesToMeters(0.1),
                                                            180,
                                                            0,
                                                            self._color_intake)  # relative angle 180 to go back up arm

        self.intake_indicator = self.intake_spacer.appendLigament("intake_indicator",
                                                                  inchesToMeters(2),
                                                                  90,
                                                                  8,
                                                                  self._color_intake)  # 90 relative to spacer

    def _init_hopper(self):
        """
        Storage area for the fuel   (TODO: does not exist yet. All measurements are a guess)
        """
        # Attached to chassis middle
        # Using a root here to decouple from chassis rotation if needed, or just attach to chassis
        # Horizontal conveyor
        # Butt up against intake (Right, X=32) and go Left (180 deg)
        self.root_hopper = self._mech.getRoot(
            "hopper_root",
            self._start_x + constants.ROBOT_CHASSIS_WIDTH - inchesToMeters(2),
            inchesToMeters(5))

        self.abs_hopper = 180
        self.hopper_length = inchesToMeters(16)
        self.hopper_tower = self.root_hopper.appendLigament("hopper_belt",
                                                            self.hopper_length,
                                                            self._get_rel_angle(self.abs_hopper,
                                                                                0),
                                                            self._line_weight,
                                                            self._color_hopper)

        # Animation: A spacer that grows to push the indicator along the belt
        self.hopper_anim_dist = 0
        self.hopper_spacer = self.root_hopper.appendLigament("hopper_spacer",
                                                             inchesToMeters(0.1),
                                                             self._get_rel_angle(self.abs_hopper,
                                                                                 0),
                                                             0,  # Line width 0 = invisible
                                                             self._color_hopper)

        self.hopper_indicator = self.hopper_spacer.appendLigament("hopper_indicator",
                                                                  inchesToMeters(2),
                                                                  self._get_rel_angle(90,
                                                                                      self.abs_hopper),
                                                                  6,
                                                                  self._color_hopper)  # 90 abs (Up)

    def _init_indexer(self):
        """
        Feeder mechanism   (TODO: does not exist yet. All measurements are a guess)
        """
        # Attached to top of indexer
        # Continue left (Relative 0 -> Absolute 180)
        self.abs_indexer = 135
        self.indexer_length = inchesToMeters(5)
        self.indexer_wheel = self.hopper_tower.appendLigament("indexer_wheel",
                                                              self.indexer_length,
                                                              self._get_rel_angle(self.abs_indexer,
                                                                                  self.abs_hopper),
                                                              self._line_weight,
                                                              self._color_indexer)
        # Animation: Spacer and Indicator
        self.indexer_anim_dist = 0
        self.indexer_spacer = self.hopper_tower.appendLigament("indexer_spacer",
                                                               inchesToMeters(0.1),
                                                               self._get_rel_angle(self.abs_indexer,
                                                                                   self.abs_hopper),
                                                               0,
                                                               self._color_indexer)
        self.indexer_indicator = self.indexer_spacer.appendLigament("indexer_indicator",
                                                                    inchesToMeters(2),
                                                                    -90,
                                                                    8,
                                                                    self._color_indexer)  # -90 relative to indexer

    def _init_shooter(self):
        """
        Flywheel mechanism.
        """
        # Attached to indexer or separate tower   (TODO: does not exist yet. All measurements are a guess)
        # Left of indexer, angling up/left
        # Parent is Indexer (135). Target is 70 (pointing up/rightish).
        self.abs_shooter = 70
        self.shooter_hood = self.indexer_wheel.appendLigament("shooter_hood",
                                                              inchesToMeters(8),
                                                              self._get_rel_angle(self.abs_shooter,
                                                                                  self.abs_indexer),
                                                              self._line_weight,
                                                              self._color_shooter)
        # Flywheel (White) - attached to center of shooter
        # Use a spacer to get to the middle (length 4)
        self.shooter_center_spacer = self.indexer_wheel.appendLigament("shooter_center_spacer",
                                                                       inchesToMeters(4),
                                                                       self._get_rel_angle(self.abs_shooter,
                                                                                           self.abs_indexer),
                                                                       0,
                                                                       self._color_shooter)

        self.flywheel_ligament = self.shooter_center_spacer.appendLigament("flywheel",
                                                                           inchesToMeters(2),
                                                                           0,
                                                                           6,
                                                                           self._color_wheel)

        self.flywheel_ligament_2 = self.shooter_center_spacer.appendLigament("flywheel_2",
                                                                             inchesToMeters(2),
                                                                             180,
                                                                             6,
                                                                             self._color_wheel)

    def _init_climber(self):
        """
        Extension arms."""
        # Attached to chassis center/side
        self.climber_root_y = inchesToMeters(2)
        self.root_climber = self._mech.getRoot("climber_root",
                                               self._start_x + inchesToMeters(11),  # TODO: verify location
                                               self.climber_root_y)
        self.abs_climber = 90
        self.climber_stage_1 = self.root_climber.appendLigament("climber_stage_1",
                                                                inchesToMeters(15),
                                                                self._get_rel_angle(self.abs_climber,
                                                                                    0),
                                                                self._line_weight,
                                                                self._color_climber)

        # Hook pointing left (180 abs)
        self.abs_hook = 180
        self.climber_hook = self.climber_stage_1.appendLigament("climber_hook",  # TODO: verify size
                                                                inchesToMeters(4),
                                                                self._get_rel_angle(self.abs_hook,
                                                                                    self.abs_climber),
                                                                self._line_weight,
                                                                self._color_wheel)

    def _init_fuel(self):
        """
        A game piece sitting in front of the robot
        """
        # Start on floor (Y=3 for 6" diam), to the right of intake
        # Intake tip is approx X=39 (start_x + 27 + ~7). Place fuel at X=45 (start_x + 40).
        self.fuel_root = self._mech.getRoot("fuel_root",
                                            self._start_x + constants.ROBOT_CHASSIS_WIDTH + inchesToMeters(7),
                                            inchesToMeters(3))

        # "just make one 6" tall 6" wide bar"
        # Root is at center (Y=3). Go down 3 to bottom, then draw up 6.

        # Invisible spacer to get to bottom
        self.fuel_spacer = self.fuel_root.appendLigament("fuel_spacer",
                                                         inchesToMeters(3),
                                                         -90,
                                                         0,
                                                         self._color_fuel)

        self.fuel_ligament = self.fuel_spacer.appendLigament("fuel_solid",
                                                             inchesToMeters(6),
                                                             180,
                                                             20,
                                                             self._color_fuel)

    def periodic(self) -> None:
        """
        Update the mechanisms, but not if we are in a competition. Also do not
        update it during simulation, since simulation Periodic does that for us.
        """
        if self._robot.isEnabled() and not DriverStation.isFMSAttached() and not RobotBase.isSimulation():
            self._update_drivetrain()

            if self._container.intake_pivot is not None:
                self._update_intake()

            self._update_hopper()

            if self._container.indexer is not None:
                self._update_indexer()

            if self._container.flywheel is not None:
                self._update_shooter()

            if self._container.climber is not None:
                self._update_climber()

    def sim_init(self, physics_controller: 'PhysicsInterface') -> None:
        """
        Initialize any simulation only needed parameters
        """
        self._physics_controller = physics_controller
        # TODO: Anything

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be updated.
        This function is called from the '_simulationPeriodic' function of the
        robotpy core routine and is called at a period >= 10 mS. Note that the
        CommandScheduler also has an 'simulationPeriodic' function that it calls
        into all Command2 based subsystems at its update period which has a
        default rate of 20 mS.

        This is called 'after' the CommandScheduler's 'simulationPeriodic', so if
        that function uses pykit's logging method, you should use those values in
        your simulation.

        :param now:     The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        pass

    def simulationPeriodic(self) -> None:
        """
        This method is called periodically by the CommandScheduler (after the periodic
        function). It is useful for updating subsystem-specific state that needs to be
        maintained for simulations, such as for updating simulation classes and setting
        simulated sensor readings.

        Unlike the physics 'update_sim', it is not called with the current time (now)
        or the amount of time since 'update_sim' was called (tm_diff).  It is called
        just after the 'periodic' call and before the 'update_sim' is called. One other
        'important' difference is 'update_sim' is called at a period >= 10 ms instead
        of the default 20 mS for the CommandScheduler's simulationPeriodic (this function).
        """
        if self._robot.isEnabled():
            LogTracer.resetOuter(f"{self.getName()}-simulationPeriodic")

            self._update_drivetrain()

            if self._container.intake_pivot is not None:
                self._update_intake()

            self._update_hopper()

            if self._container.indexer is not None:
                self._update_indexer()

            if self._container.flywheel is not None:
                self._update_shooter()

            if self._container.climber is not None:
                self._update_climber()

            self._update_fuel()
            LogTracer.recordTotal()

    def _update_drivetrain(self) -> None:
        # Speed (m/s) -> angular velocity.
        # Just adding a factor to animate it.

        chassis_speed: ChassisSpeeds = self._container.robot_drive.chassis_speeds()
        speed: meters_per_second = max(math.fabs(chassis_speed.vx), math.fabs(chassis_speed.vy))
        rotation_step = speed * 10  # arbitrary scaling

        new_rear_angle = self.rear_wheel_ligament.getAngle() + rotation_step
        self.rear_wheel_ligament.setAngle(new_rear_angle)
        self.rear_wheel_ligament_2.setAngle(new_rear_angle + 180)

        new_front_angle = self.front_wheel_ligament.getAngle() + rotation_step
        self.front_wheel_ligament.setAngle(new_front_angle)
        self.front_wheel_ligament_2.setAngle(new_front_angle + 180)

    def _update_intake(self) -> None:
        # Pivot relative to the vertical post (which is at 90 absolute).
        angle: degrees = self._adjust_intake_angle(self._container.intake_pivot.angle)
        speed: degrees_per_second = self._container.intake_pivot.angular_velocity

        # Note: appendLigament angle is relative to parent. Parent is 90 deg (vertical).
        self.intake_arm.setAngle(self._get_rel_angle(angle, self._abs_intake_post_angle))

        # Animate bar moving along the intake arm
        # if abs(speed) > 0:
        #     self.intake_anim_dist += abs(speed) * inchesToMeters(0.2)
        #
        #     if self.intake_anim_dist > self.intake_arm_length:
        #         self.intake_anim_dist = 0
        #
        #     self.intake_spacer.setLength(self.intake_anim_dist)

    def _update_hopper(self) -> None:
        # Animate the ball moving left (at just below indexer speed)
        speed: meters_per_second = 0.0  # TODO: figure this out

        if abs(speed) > 0:
            self.hopper_anim_dist += speed * inchesToMeters(0.2)  # Scale speed for animation

            if self.hopper_anim_dist > self.hopper_length:
                self.hopper_anim_dist = 0

            self.hopper_spacer.setLength(self.hopper_anim_dist)

    def _update_indexer(self) -> None:
        # Animate bar moving up the indexer
        speed: meters_per_second = 0.0  # TODO: figure this out

        if abs(speed) > 0:
            self.indexer_anim_dist += speed * inchesToMeters(0.1)

            if self.indexer_anim_dist > self.indexer_length:
                self.indexer_anim_dist = 0

            if self.indexer_anim_dist < 0:
                self.indexer_anim_dist = self.indexer_length

            self.indexer_spacer.setLength(self.indexer_anim_dist)

    def _update_shooter(self) -> None:
        # Visualize speed by color or spinning ligament
        rpm: revolutions_per_minute = 0.0  # TODO: Figure this out

        if abs(rpm) > 10:
            rotation_step = rpm / 100  # Scale RPM to rotation per frame
            new_angle = self.flywheel_ligament.getAngle() + rotation_step
            self.flywheel_ligament.setAngle(new_angle)
            self.flywheel_ligament_2.setAngle(new_angle + 180)

    def _update_climber(self) -> None:
        # Calculate length: Target Height - Root Height
        height_from_ground: meters = 0.0  # TODO: Figure this out

        length = max(0.0, height_from_ground - self.climber_root_y)
        self.climber_stage_1.setLength(length)

    def _update_fuel(self) -> None:
        x: meters = 0.0
        y: meters = 0.0
        # self.fuel_root.setPosition(x, y)
