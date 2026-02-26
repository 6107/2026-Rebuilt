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

from typing import Optional

#import cv2
import numpy as np
import wpimath.controller
from wpilib import SmartDashboard

from lib_6107.commands.command import BaseCommand
from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem
from robot_2026.subsystems.swervedrive.drivesubsystem import DriveSubsystem


class TrackTagCommand(BaseCommand):
    def __init__(self, drivetrain: 'DriveSubsystem',
                 camera: VisionSubsystem,
                 target_id: int, rotate_only: Optional[bool] = False,
                 stop_when_done: Optional[bool] = True):
        """
        Track an Apriltag around the room.  If the target ID is None, then use the best
        apriltag detection algorithm. If 'rotate_only' is true, then the robot will turn
        in place.
        """
        super().__init__(drivetrain)
        self._drivetrain = drivetrain
        self._target_id = target_id
        self._best_target_id = target_id
        self._rotate_only = rotate_only
        self._stop_when_done = stop_when_done

        # Camera Setup
        self.camera: VisionSubsystem = camera
        # self.detector = robotpy_apriltag.AprilTagDetector()
        # self.detector.addFamily("tag36h11")

        # PID Controller for turning (Adjust P, I, D based on robot)
        self._rotation_pid = wpimath.controller.PIDController(0.05, 0.0, 0.0)
        self._rotation_pid.setSetpoint(0)  # We want 0 error (center of image)

        self.addRequirements(drivetrain)

        # Pre-allocate image buffer
        self._frame = np.zeros(shape=(480, 640, 3), dtype=np.uint8)

    def execute(self):
        """
        The initial subroutine of a command. Called once when the command is initially scheduled.

        Attempt initial target acquisition. If not found, begin rotation to scan the fieldd
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
                turn_speed = self._rotation_pid.calculate(error)
                self._drivetrain.arcade_drive(0, turn_speed)

                target_found = True
                SmartDashboard.putNumber("Tag Error", error)
                break

        if not target_found:
            # Stop or search behavior if tag not found
            self._drivetrain.arcade_drive(0, 0)

    def isFinished(self):
        """
        Keep running until trigger (or associated button) is released

        If the target ID is None, then we are free to choose the best target
        that is loaded for our camera to find.  If there is a better one, then
        switch to it and start tracking that one, if there is one.
        """
        if self._target_id is None or self._best_target_id is None:
            pass
            pass  # Todo, look for any tag and pick the best
            pass

        return False

    def end(self, interrupted: bool) -> None:
        """
        Hey, I've been working hard at this, and you are just now giving up!
        Okay, I will just stop at where I'm at...
        """
        if self._stop_when_done:
            self._drivetrain.stop()

        super().end(interrupted)
