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

import wpimath.controller
import robotpy_apriltag

from commands2 import Command
from cscore import CameraServer
import cv2
import numpy as np
from wpilib import SmartDashboard
from subsystems.swervedrive.drivesubsystem import DriveSubsystem
from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem

class TrackTagCommand(Command):
    def __init__(self, drivetrain: 'DriveSubsystem',
                 camera: VisionSubsystem,
                 target_id: int):
        """
        Track an Apriltag around the room.  If the target ID is None, then use the best
        apriltag detection algorithm.
        """
        super().__init__()
        self._drivetrain = drivetrain
        self._target_id = target_id

        # Camera Setup
        self.camera = CameraServer.getVideo(name=camera_name)
        self.detector = robotpy_apriltag.AprilTagDetector()
        self.detector.addFamily("tag36h11")

        # PID Controller for turning (Adjust P, I, D based on robot)
        self._pid = wpimath.controller.PIDController(0.05, 0.0, 0.0)
        self._pid.setSetpoint(0)  # We want 0 error (center of image)

        self.addRequirements(drivetrain)

        # Pre-allocate image buffer
        self._frame = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    def execute(self):
        """
        The initial subroutine of a command. Called once when the command is initially scheduled.
        """
        timestamp, self._frame = self.camera.grabFrame(self._frame)
        if timestamp == 0: return  # No frame

        gray = cv2.cvtColor(self._frame, cv2.COLOR_BGR2GRAY)
        results = self.detector.detect(gray)

        target_found = False
        for result in results:
            if result.getId() == self._target_id:
                # Get center X pixel coordinate
                center_x = result.getCenter().x
                # Assuming 640x480 resolution, center is 320
                error = center_x - 320

                # Calculate turn speed
                turn_speed = self._pid.calculate(error)
                self._drivetrain.arcadeDrive(0, turn_speed)

                target_found = True
                SmartDashboard.putNumber("Tag Error", error)
                break

        if not target_found:
            # Stop or search behavior if tag not found
            self._drivetrain.arcade_drive(0, 0)

    def isFinished(self):
        """
        Keep running until trigger (or associated button) is released
        """
        return False

    def end(self, interrupted: bool) -> None:
        self._drivetrain.stop()
        super().end(interrupted)