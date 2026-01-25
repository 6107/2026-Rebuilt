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

import logging
from typing import Optional

from commands2 import Subsystem
from ntcore import NetworkTableInstance
from wpilib import Timer, SmartDashboard

logger = logging.getLogger(__name__)


class LimelightCamera(Subsystem):
    def __init__(self, container: 'RobotContainer', name: Optional[str] = "limelight") -> None:

        super().__init__()

        self.name = name
        self._robot = container.robot

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable(self.name)
        self._path = self.table.getPath()

        self.pipelineIndexRequest = self.table.getDoubleTopic("pipeline").publish()
        self.pipelineIndex = self.table.getDoubleTopic("getpipe").getEntry(-1)
        # "cl" and "tl" are additional latencies in milliseconds

        self.ledMode = self.table.getIntegerTopic("ledMode").getEntry(-1)
        self.camMode = self.table.getIntegerTopic("camMode").getEntry(-1)
        self.tx = self.table.getDoubleTopic("tx").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("ty").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("ta").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("hb").getEntry(0)

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False
        self.ticked = False

        self.localizerSubscribed = False

    def addLocalizer(self):
        if self.localizerSubscribed:
            return

        self.localizerSubscribed = True
        # if we want MegaTag2 localizer to work, we need to be publishing two things (to the camera):
        #   1. what robot's yaw is ("yaw=0 degrees" means "facing North", "yaw=90 degrees" means "facing West", etc.)
        #   2. where is this camera sitting on the robot (e.g. y=-0.2 meters to the right, x=0.1 meters fwd from center)
        self.robotOrientationSetRequest = self.table.getDoubleArrayTopic("robot_orientation_set").publish()
        self.cameraPoseSetRequest = self.table.getDoubleArrayTopic("camerapose_robotspace_set").publish()
        self.imuModeRequest = self.table.getIntegerTopic("imumode_set").publish()  # this is only for Limelight 4

        # and we can then receive the localizer results from the camera back
        self.botPose = self.table.getDoubleArrayTopic("botpose_orb_wpiblue").getEntry([])
        self.botPoseFlipped = self.table.getDoubleArrayTopic("botpose_orb_wpired").getEntry([])

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(float(index))

    def getPipeline(self) -> int:
        return int(self.pipelineIndex.get(-1))

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if self.getX() != 0.0 and self.heartbeating:
            return True

    def getSecondsSinceLastHeartbeat(self) -> float:
        return Timer.getFPGATimestamp() - self.lastHeartbeatTime

    def periodic(self) -> None:
        now = Timer.getFPGATimestamp()
        heartbeat = self.getHB()

        if heartbeat != self.lastHeartbeat:
            self.lastHeartbeat = heartbeat
            self.lastHeartbeatTime = now

        heartbeating = now < self.lastHeartbeatTime + 5  # no heartbeat for 5s => stale camera
        if heartbeating != self.heartbeating:
            logger.warning(f"Camera {self.name}: {'UPDATING' if heartbeating else 'NO LONGER UPDATING'}")

        self.heartbeating = heartbeating

        self.dashboard_periodic()

    def dashboard_initialize(self) -> None:
        """
        Configure the SmartDashboard for this subsystem
        """
        # SmartDashboard.putData("Field", self.field)
        SmartDashboard.putString('Camera/name', self.name)
        SmartDashboard.putString('Camera/type', "Limelight")

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        if update_dash:
            SmartDashboard.putString('Camera/heartbeat', "Alive" if self.heartbeating else "Dead")
            SmartDashboard.putNumber('Camera/last-heartbeat', self.lastHeartbeatTime)
