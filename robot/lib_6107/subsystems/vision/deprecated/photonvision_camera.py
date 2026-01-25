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


class PhotonVisionCamera(Subsystem):
    def __init__(self, container: 'RobotContainer', name: Optional[str] = "photonvision") -> None:
        super().__init__()

        self.name = name
        self._robot = container.robot

        instance = NetworkTableInstance.getDefault()
        self.table = instance.getTable("photonvision").getSubTable(self.name)
        self._path = self.table.getPath()

        # TODO: If the reset of this is similar or identical to LimeLight, see about doing
        #       a base class for a camera subsystem

        self.pipelineIndexRequest = self.table.getIntegerTopic("pipelineIndexRequest").publish()
        self.pipelineIndex = self.table.getIntegerTopic("pipelineIndexState").getEntry(-1)

        self.driverModeRequest = self.table.getBooleanTopic("driverModeRequest").publish()
        self.driverMode = self.table.getBooleanTopic("driverMode").getEntry(False)

        self.tx = self.table.getDoubleTopic("targetYaw").getEntry(0.0)
        self.ty = self.table.getDoubleTopic("targetPitch").getEntry(0.0)
        self.ta = self.table.getDoubleTopic("targetArea").getEntry(0.0)
        self.hb = self.table.getIntegerTopic("heartbeat").getEntry(0)
        self.hasTarget = self.table.getBooleanTopic("hasTarget").getEntry(False)
        self.latencyMillis = self.table.getDoubleTopic("latencyMillis").getEntry(0)

        self.lastHeartbeat = 0
        self.lastHeartbeatTime = 0
        self.heartbeating = False

    def setPipeline(self, index: int):
        self.pipelineIndexRequest.set(index)

    def getPipeline(self) -> int:
        return self.pipelineIndex.get(-1)

    def setDriverMode(self, enabled: bool):
        self.driverModeRequest.set(enabled)

    def getDriverMode(self) -> bool:
        return self.driverMode.get(False)

    def getA(self) -> float:
        return self.ta.get()

    def getX(self) -> float:
        return self.tx.get()

    def getY(self) -> float:
        return self.ty.get()

    def getHB(self) -> float:
        return self.hb.get()

    def hasDetection(self):
        if self.hasTarget.get() and self.heartbeating:
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
        SmartDashboard.putString('Camera/type', "PhotonVision")

    def dashboard_periodic(self) -> None:
        """
        Called from periodic function to update dashboard elements for this subsystem
        """
        divisor = 10 if self._robot.isEnabled() else 20
        update_dash = self._robot.counter % divisor == 0

        if update_dash:
            SmartDashboard.putString('Camera/heartbeat', "Alive" if self.heartbeating else "Dead")
            SmartDashboard.putNumber('Camera/last-heartbeat', self.lastHeartbeatTime)
