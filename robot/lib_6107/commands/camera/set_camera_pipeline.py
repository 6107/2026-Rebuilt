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
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import logging

import commands2

logger = logging.getLogger(__name__)


class SetCameraPipeline(commands2.Command):

    def __init__(self, camera, pipelineIndex=0, onlyTagIds=()):
        super().__init__()

        self.pipelineIndex = pipelineIndex
        self.onlyTagIds = onlyTagIds
        self.camera = camera
        self.addRequirements(camera)

    def initialize(self) -> None:
        # if camera allows to set filter to look for specific tag IDs, filter for them
        if hasattr(self.camera, "setOnlyTagIds"):
            self.camera.setOnlyTagIds(self.onlyTagIds)

        # if camera has "setPipeline", set it
        if hasattr(self.camera, "setPipeline"):
            self.camera.setPipeline(self.pipelineIndex)

    def isFinished(self) -> bool:
        # if camera has no "setPipeline", we have nothing to wait for
        if not hasattr(self.camera, "setPipeline"):
            return True

        # we are finished when the camera has responded that pipeline index is now set
        if self.camera.getPipeline() == self.pipelineIndex:
            return True

        # we are in sim, and camera doesn't respond
        if commands2.TimedCommandRobot.isSimulation():
            return True

        # otherwise, print that we aren't finished
        logging.info(
            f"SetCameraPipeline: not yet finished, because camera pipeline = {self.camera.getPipeline()} and we want {self.pipelineIndex}")

        return False
