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

try:
    import logging

    from typing import Optional

    from robotpy_apriltag import AprilTagField
    from wpimath.geometry import Transform3d
    from wpimath.units import milliseconds, seconds

    from lib_6107.subsystems.vision.deprecated.limelight_camera import LimelightCamera
    from lib_6107.subsystems.vision.deprecated.limelight_localizer import LimelightLocalizer

    from lib_6107.subsystems.vision.visionsubsystem import VisionSubsystem, VisionTargetData

    logger = logging.getLogger(__name__)


    class LimelightVisionSubsystem(VisionSubsystem):
        def __init__(self, name: str, field: AprilTagField, transform: Transform3d, drivetrain: 'DriveSubsystem'):
            super().__init__(name, field, transform, drivetrain)
            self._camera = None

        @property
        def latency(self) -> Optional[milliseconds]:
            return None

        @property
        def timestamp(self) -> Optional[seconds]:
            """
            Returns the estimated time the frame was taken, in the Received system's time base
            """
            return None

        @property
        def best_target(self) -> Optional[VisionTargetData]:
            """
            Returns the best target in this pipeline result. If there are no targets, this method will
            return null. The best target is determined by the target sort mode in the PhotonVision UI.
            """
            return None

        def periodic(self) -> None:
            pass

        def simulationPeriodic(self):
            pass


except ImportError:
    pass
