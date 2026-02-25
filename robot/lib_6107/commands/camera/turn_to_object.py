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
#
from lib_6107.commands.drivetrain.aimtodirection import AimToDirection


def turn_to_object(camera, drivetrain) -> None:
    """
    This command is used to have the robot camera rotate the object in the
    direction of an object

    If you want the robot to slowly chase that object... replace the 'self.rotate'
    line below with: self.arcadeDrive(0.1, turn_speed)

    """
    x = camera.getX()
    turn_speed = -0.01 * x
    drivetrain.rotate(turn_speed)
