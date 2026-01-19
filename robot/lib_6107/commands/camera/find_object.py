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
import commands2
from commands.aimtodirection import AimToDirection


class FindObject(commands2.Command):

    def __init__(self, camera, drivetrain, turnDegrees=-45, turnSpeed=1.0, waitSeconds=0.1):
        super().__init__()
        self.camera0 = camera
        self.drivetrain = drivetrain
        self.addRequirements(camera)
        self.addRequirements(drivetrain)

        # to find object, we will be running (wait + turn) + (wait + turn) ... in a long cycle, until object is detected
        wait = commands2.WaitCommand(waitSeconds)
        turn = AimToDirection(
            degrees=lambda: self.drivetrain.heading.degrees() + turnDegrees,
            drivetrain=self.drivetrain,
            speed=turnSpeed
        )

        self.subcommand = wait.andThen(turn)

    def initialize(self):
        self.subcommand.initialize()

    def isFinished(self) -> bool:
        return self.camera0.hasDetection()

    def execute(self):
        # 1. just execute the wait+turn subcommand
        self.subcommand.execute()

        # 2. and if that command is finished, start it again (we are doing cycles, right?)
        if self.subcommand.isFinished():
            self.subcommand.end(False)
            self.subcommand.initialize()

    def end(self, interrupted: bool):
        self.subcommand.end(interrupted)
