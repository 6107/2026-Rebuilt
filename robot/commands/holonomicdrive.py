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
# From Gene Panov's (Team 714) CommandRevSwerve project (and FRC Python videos)

import logging
from typing import Optional

import commands2
from wpimath import applyDeadband

logger = logging.getLogger(__name__)


class HolonomicDrive(commands2.Command):
    """
    "holonomic" means that it can rotate independently of driving forward or left
    (examples: mecanum drivetrain, ball drivetrain, swerve drivetrain)
    """

    def __init__(self, robot_container, drivetrain, forwardSpeed, leftSpeed, rotationSpeed, deadband=0,
                 field_relative: Optional[bool] = True, rate_limit: bool = False, square: bool = False):
        """
        Drive the robot at `driveSpeed` and `rotationSpeed` until this command is terminated.
        """
        super().__init__()

        self.robot = robot_container.robot
        self.container = robot_container

        self.forwardSpeed = forwardSpeed     # TODO: Make sure parameters are meters_per_second and radians_per_second
        if not callable(forwardSpeed):
            self.forwardSpeed = lambda: forwardSpeed

        self.leftSpeed = leftSpeed
        if not callable(leftSpeed):
            self.leftSpeed = lambda: leftSpeed

        self.rotationSpeed = rotationSpeed
        if not callable(rotationSpeed):
            self.rotationSpeed = lambda: rotationSpeed

        assert deadband >= 0, f"deadband={deadband} is not positive"
        self.deadband = deadband

        self.drivetrain = drivetrain
        self.rate_limit = rate_limit
        self.square = square

        self.field_relative = field_relative

        self.addRequirements(drivetrain)

    def initialize(self):
        pass

    def isFinished(self) -> bool:
        return False  # never finishes, you should use it with "withTimeout(...)"

    def execute(self):
        forward_speed = self.forwardSpeed()
        left_speed = self.leftSpeed()
        rotation_speed = self.rotationSpeed()

        flipped = self.field_relative and self.container.is_red_alliance
        if flipped:
            forward_speed, left_speed = -forward_speed, -left_speed

        if self.robot.isEnabled() and self.robot.counter % 20 == 0:
            logger.debug(
                f"HolonomicDrive command: forward={forward_speed}, left={left_speed}, rotation={left_speed}, deadband={self.deadband}, flipped: {flipped}")

        # TODO: Make sure parameters are meters_per_second and radians_per_second
        self.drivetrain.drive(applyDeadband(forward_speed, self.deadband),
                              applyDeadband(left_speed, self.deadband),
                              applyDeadband(rotation_speed, self.deadband),
                              self.field_relative, self.rate_limit, self.square)

    def end(self, interrupted: bool):
        self.drivetrain.stop()  # stop immediately if command is ending
