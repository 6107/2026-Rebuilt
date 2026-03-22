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
# See the documentation for more details on how this works
#
# Documentation can be found at https://robotpy.readthedocs.io/projects/pyfrc/en/latest/physics.html
#
# The idea here is you provide a simulation object that overrides specific
# pieces of WPILib, and modifies motors/sensors accordingly depending on the
# state of the simulation. An example of this would be measuring a motor
# moving for a set period of time, and then changing a limit switch to turn
# on after that period of time. This can help you do more complex simulations
# of your robot code without too much extra effort.
#
# Examples can be found at https://github.com/robotpy/examples
import logging

from pyfrc.physics.core import PhysicsInterface

from robot import MyRobot
from robot_2026.field.field_2026 import BLUE_TEST_POSE, RED_TEST_POSE

logger = logging.getLogger(__name__)


class PhysicsEngine:
    """
    Simulates a 2-wheel XRP robot using Arcade Drive joystick control.

    Any objects created or manipulated in this file are for simulation purposes only.
    """
    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        Initialize the simulator.  This method is called after the container and all
        subsystems have been initialized.

        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """
        logger.info("PhysicsEngine.__init__: entry")

        self._physics_controller = physics_controller
        self._robot: MyRobot = robot

        # Start up network tables server. This will listen on all ports   TODO: See if this is needed in simulation
        # inst = NetworkTableInstance.getDefault()
        # inst.startServer()

        # Initialize our simulated subsystems
        for subsystem in robot.container.subsystems:
            if hasattr(subsystem, "sim_init") and callable(getattr(subsystem, "sim_init")):
                subsystem.sim_init(physics_controller)

        # Set up field, it is declared in the physics controller simulation file
        # and initialized in the _simulationInit() method and it initializes teh
        # SmartDashboard.
        self.field = physics_controller.field

        # Register for any changes in alliance before the match starts
        robot.container.register_alliance_change_callback(self._alliance_change)
        self._alliance_change(self._robot.container.is_red_alliance,
                              self._robot.container.alliance_location)

        # TODO: If vision odometry is supported in simulation, this may need to be
        #       changed to the robot's field view and not the 'overhead' view of the
        #       playing field.

        logger.info("PhysicsEngine.__init__: exit")

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

        This routine will scan all subsystems and if it contains an 'update_sim'
        function, it will be called.

        :param now:     The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """
        for subsystem in self._robot.container.subsystems:
            if hasattr(subsystem, "update_sim") and callable(getattr(subsystem, "update_sim")):
                subsystem.update_sim(self, now, tm_diff)

    def _alliance_change(self, is_red: bool, location: int) -> None:
        """
        Called whenever the alliance changes colors before the match / competition begins
        """
        initial_pose = RED_TEST_POSE[location] if is_red else BLUE_TEST_POSE[location]
        self._physics_controller.field.setRobotPose(initial_pose)
