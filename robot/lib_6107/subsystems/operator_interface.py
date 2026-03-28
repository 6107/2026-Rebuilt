from math import copysign
from typing import Callable

from commands2 import cmd, Command
from commands2.button import CommandPS4Controller, CommandXboxController
from wpilib.interfaces import GenericHID

AnalogInput = Callable[[], float]

kXboxJoystickDeadband = 0.1
"""dimensionless"""

kKeyboardJoystickDeadband = 0.0
"""dimensionless"""

kXboxTriggerActivationThreshold = 0.5


def deadband(input_fn: AnalogInput, deadband_limit: float) -> AnalogInput:
    def _withDeadband() -> float:
        value = input_fn()
        if abs(value) <= deadband_limit:
            return 0
        else:
            return value

    return _withDeadband


def invert(input_fn: AnalogInput) -> AnalogInput:
    def _invert() -> float:
        return -1 * input_fn()

    return _invert


def sign_square(input_fn: AnalogInput) -> AnalogInput:
    def square() -> float:
        val = input_fn()
        return copysign(val * val, val)

    return square


class OperatorInterface:
    """
    The controls that the operator(s)/driver(s) interact with
    """

    driverController: CommandPS4Controller | CommandXboxController
    operatorController: CommandPS4Controller | CommandXboxController

    def __init__(self) -> None:
        # if kRobotMode == RobotModes.SIMULATION:
        #     self.driverController = CommandXboxController(0)
        # else:
        self.driverController = CommandXboxController(0)
        self.operatorController = CommandXboxController(1)

        self.driverX = sign_square(invert(deadband(self.driverController.getLeftX, kXboxJoystickDeadband)))
        self.driverY = sign_square(invert(deadband(self.driverController.getLeftY, kXboxJoystickDeadband)))
        self.driverRotationX = invert(deadband(self.driverController.getRightX, kXboxJoystickDeadband))
        self.driverRotationY = invert(deadband(self.driverController.getRightY, kXboxJoystickDeadband))

    def rumble_controllers(self, amount: float = 1.0) -> None:
        self.driverController.setRumble(GenericHID.RumbleType.kBothRumble, amount)
        self.operatorController.setRumble(GenericHID.RumbleType.kBothRumble, amount)

    def rumble_controllers_command(self, amount: float = 1.0) -> Command:
        return cmd.startEnd(
            lambda: self.rumble_controllers(amount),
            lambda: self.rumble_controllers(0.0)
        )
