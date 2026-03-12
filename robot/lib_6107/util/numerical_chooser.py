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
from typing import Optional, SupportsFloat, SupportsInt

from commands2 import Subsystem
from wpiutil import Sendable, SendableBuilder, SendableRegistry

logger = logging.getLogger(__name__)


# TODO: The pykit LoggedSendableChooser allows for a type to be specified
class IntegerEditBox(Subsystem):
    def __init__(self, name: str,
                 initial_value: Optional[int] = None,
                 minimum_value: Optional[int] = None,
                 maximum_value: Optional[int] = None):
        super().__init__()
        self._name: str = name
        self._value: int | None = None
        self._min: int | None = minimum_value
        self._max: int | None = maximum_value
        self.setName(name)

        # Validate that the initial value is what we accept
        self.set_value(initial_value)
        SendableRegistry.add(self, name=name)

    def __str__(self) -> str:
        return str(self._value)

    def __repr__(self):
        return f"IntegerEditBox({self._name}, value={self._value}, min={self._min}, max={self._max})"

    @property
    def name(self) -> str:
        return self._name

    def initSendable(self, builder: SendableBuilder) -> None:
        # Define how to send data to SmartDashboard
        builder.setSmartDashboardType("String Chooser")

        # Add a property that can be read and written
        builder.addIntegerProperty(self._name, lambda: self._value, lambda value: self.set_value(value))

    def set_value(self, value: SupportsInt) -> None:
        if isinstance(value, int):
            if self._min is not None and value < self._min:
                logger.error(f"{self._name} IntegerEditBox: Value too small. {value} < {self._min}")

            elif self._max is not None and value > self._max:
                logger.error(f"{self._name} IntegerEditBox: Value too large. {value} > {self._max}")

            else:
                self._value = value
        else:
            logger.error(f"{self._name} IntegerEditBox: Invalid type {type(value)}")


class FloatEditBox(Sendable):
    def __init__(self, name: str,
                 initial_value: Optional[float] = None,
                 minimum_value: Optional[float] = None,
                 maximum_value: Optional[float] = None):
        super().__init__()
        self._name: str = name
        self._value: float | None = None
        self._min: float | None = minimum_value
        self._max: float | None = maximum_value

        # Validate that the initial value is what we accept
        self.set_value(initial_value)
        # SendableRegistry.add(self, subsystem="Subsystem", name=name)

    def __str__(self) -> str:
        return str(self._value)

    def __repr__(self):
        return f"FloatEditBox({self._name}, value={self._value}, min={self._min}, max={self._max})"

    @property
    def name(self) -> str:
        return self._name

    def initSendable(self, builder: SendableBuilder) -> None:
        # Define how to send data to SmartDashboard
        builder.setSmartDashboardType("Subsystem")

        # Add a property that can be read and written
        builder.addDoubleProperty(self._name, lambda: self._value, self.set_value)

    def set_value(self, value: SupportsFloat) -> None:
        if isinstance(value, float):
            if self._min is not None and value < self._min:
                logger.error(f"{self._name} FloatEditBox: Value too small. {value} < {self._min}")

            elif self._max is not None and value > self._max:
                logger.error(f"{self._name} FloatEditBox: Value too large. {value} > {self._max}")

            else:
                self._value = value
        else:
            logger.error(f"{self._name} FloatEditBox: Invalid type {type(value)}")
