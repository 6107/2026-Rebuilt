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

import logging
import os
from typing import Callable, List, Optional, Sequence, Tuple

from robotpy_apriltag import AprilTag, AprilTagField, AprilTagFieldLayout
from wpilib import getDeployDirectory, SendableChooser, SmartDashboard
from wpimath.geometry import Pose3d
from wpimath.units import meters

# Setup Logging
logger = logging.getLogger(__name__)

FieldInfo = Sequence[Tuple[str, Optional[AprilTagField], str]]


class Field:
    """
    This class supports BLUE/RED alliances.

    When looking at the playing field, the origin is 0,0 (bottom left corner in landscape
    mode) with the Blue team on the left (lowest x-coordinate).  For the three teams in
    an alliance, we also assume

    NOTE: Any positional information will change each year based on the field. One other
          assumption is that

    All values are in meters
    """
    _field_info: FieldInfo = [["", None, ""]]

    def __init__(self):
        # First is the default
        self._april_tag_chooser = SendableChooser()
        self._april_tag_chooser.setDefaultOption(self._field_info[0][0], self._field_info[0][1])

        for field in self._field_info[1:]:
            self._april_tag_chooser.addOption(field[0], field[1])

        SmartDashboard.putData("Field Selector", self._april_tag_chooser)

        print("TODO: Support NT4 here")
        # self._chooser_entry = NetworkTables.getTable("SmartDashboard").getEntry("Field Selector/active")
        #
        # self._chooser_entry.addListener(self._on_field_changed,
        #                                 NetworkTables.NotifyFlags.UPDATE | NetworkTables.NotifyFlags.LOCAL)

        # Map AprilTagField to backup file mapping
        self._file_map = {tag_field: file for _, tag_field, file in self._field_info}

        # Callbacks when field layout changes
        self._layout_callbacks: List[Callable[[Optional[AprilTagField], Optional[AprilTagFieldLayout]], None]] = []

        # And the current field and layout
        self._field: Optional[AprilTagField] = None
        self._layout: Optional[AprilTagFieldLayout] = None

        # Set up the default field and layout
        self._init_april_tags()

    @property
    def field(self) -> Optional[AprilTagField]:
        return self._field

    @property
    def layout(self) -> Optional[AprilTagFieldLayout]:
        return self._layout

    @property
    def field_length(self) -> meters:
        """
        x maximum
        """
        return self._layout.getFieldLength() if self._layout else 0

    @property
    def field_width(self) -> meters:
        """
        y maximum
        """
        return self._layout.getFieldWidth() if self._layout else 0

    @property
    def origin(self) -> Optional[Pose3d]:
        return self._layout.getOrigin() if self._layout else None

    @property
    def tags(self) -> Optional[List[AprilTag]]:
        return self._layout.getTags() if self._layout else None

    def getTagPos(self, id: int) -> Optional[Pose3d]:
        return self._layout.getTagPose(id) if self._layout else None

    def in_blue_alliance_zone(self, x: float) -> bool:
        raise NotImplementedError("Implement in subclass")

    def in_red_alliance_zone(self, x: float) -> bool:
        raise NotImplementedError("Implement in subclass")

    def register_layout_callback(self, func: Callable[
        [Optional[AprilTagField], Optional[AprilTagFieldLayout]], None]) -> None:
        self._layout_callbacks.append(func)

    def _init_april_tags(self) -> None:
        # Load the current default and register a callback for changes
        self._load_april_tag_field(self._april_tag_chooser.getSelected())

    def _on_field_changed(self, entry, key, value, param) -> None:
        logger.info(f"Field selector changed: {entry}, {key}, {value}, {param}")
        new_field = self._april_tag_chooser.getSelected()
        self._load_april_tag_field(new_field)

    def _load_april_tag_field(self, field: AprilTagField) -> None:
        """
        Load up the selected field
        """
        existing = (self._field, self._layout)
        self._field, self._layout = field, None

        try:
            # Get from library first
            self._layout = AprilTagFieldLayout.loadField(field)
            logger.info(f"AprilTagLayout loaded for field {field}")

        except Exception as _e:
            # Fallback to directory load method
            april_tag_dir = os.path.join(getDeployDirectory(), 'fields', 'apriltags')

            if os.path.isdir(april_tag_dir) and os.access(april_tag_dir, os.R_OK):
                filename = self._file_map.get(field)

                if filename:
                    file_path = os.path.join(april_tag_dir, filename)
                    try:
                        self._layout = AprilTagFieldLayout(file_path)
                        logger.info(f"AprilTagLayout field {field} loaded from {file_path}")

                    except Exception as _e:
                        logger.warning(f"AprilTag JSON {file_path} does not exist or is not valid")
            else:
                logger.warning(f"AprilTag directory {april_tag_dir} does not exist or is not accessible")

        # Any callbacks needed
        if existing != (self._field, self._layout):
            for func in self._layout_callbacks:
                func(self._field, self._layout)
