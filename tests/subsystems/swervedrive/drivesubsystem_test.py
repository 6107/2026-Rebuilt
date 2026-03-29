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

from generated.tuner_constants import TunerConstants
from pyfrc.test_support.controller import TestController

from robot import constants, MyRobot


def test_module_offsets(control: TestController, robot: MyRobot):
    """
    While a swerve drive can drive in any direction, the notion of front/back/left/right
    still exists, and we give offsets to these based off of the center of the robot.  So
    check that they are correct polarity +/-
    """
    with control.run_robot():
        container = robot.container
        assert container is not None, "Robot Container not initialized"

        # TODO: Following '/2' assumes fairly symmetrical robot and wheel placement

        assert 0.0 < TunerConstants._front_left_x_pos < container._robot_x_width / 2
        assert 0.0 < TunerConstants._front_left_y_pos < container._robot_y_width / 2

        assert 0.0 < TunerConstants._front_right_x_pos < container._robot_x_width / 2
        assert -container._robot_y_width / 2 < TunerConstants._front_right_y_pos < 0.0

        assert -container._robot_x_width / 2 < TunerConstants._back_left_x_pos < 0.0
        assert 0.0 < TunerConstants._back_left_y_pos < container._robot_y_width / 2

        assert -container._robot_x_width / 2 < TunerConstants._back_right_x_pos < 0.0
        assert -container._robot_y_width / 2 < TunerConstants._back_right_y_pos < 0.0


def test_no_duplicate_can_bus_ids(control: TestController, robot: MyRobot):
    """
    Run through our constants and make sure they are unique and match up
    """
    power_module_id = [1]
    drive_ids = [TunerConstants._front_left_drive_motor_id, TunerConstants._front_left_steer_motor_id,
                 TunerConstants._front_left_encoder_id,
                 TunerConstants._front_right_drive_motor_id, TunerConstants._front_right_steer_motor_id,
                 TunerConstants._front_right_encoder_id,
                 TunerConstants._back_left_drive_motor_id, TunerConstants._back_left_steer_motor_id,
                 TunerConstants._back_left_encoder_id,
                 TunerConstants._back_right_drive_motor_id, TunerConstants._back_right_steer_motor_id,
                 TunerConstants._back_right_encoder_id,
                 TunerConstants._pigeon_id]
    shooter_ids = [constants.DeviceID.SHOOTER_DEVICE_ID]
    intake_ids = [constants.DeviceID.INTAKE_DEVICE_ID]
    climber_ids = [constants.DeviceID.CLIMBER_DEVICE_ID]

    all_ids = power_module_id + drive_ids + shooter_ids + intake_ids + climber_ids

    assert len(all_ids) == len(set(all_ids)), f"Duplicate IDs found: All: {all_ids}, Unique: {set(all_ids)}"


def test_robot_init_successful(control: TestController, robot: MyRobot):
    # run_robot will cause the robot to be initialized and robotInit to be called
    with control.run_robot():
        assert robot.container is not None, "Robot Container not initialized"
        assert robot.disabledTimer is not None, "Robot DisableTimer not initialized"
        assert robot.field is not None, "Robot Field not initialized"

        # Container related
        container = robot.container
        assert container._max_speed > 0
        assert container._max_angular_rate > 0

        assert container.robot_drive is not None, "Drive is not initialized"

        assert container.flywheel is not None, "Flywheel is not initialized"


def test_operator_control(control: TestController, robot: MyRobot):
    """
    This test checks to see if a joystick input causes a corresponding
    motor output. Obviously this is a silly example, but you can build
    upon it to make much more comprehensive (and stateful) tests.

    Keep in mind that when you set a value, the robot code does not
    see the value immediately, but only when teleopPeriodic is called,
    which is typically every 20ms
    """

    # run_robot will cause the robot to be initialized and robotInit to be called
    # with control.run_robot():
    #     motorsim = wpilib.simulation.PWMSim(robot.motor.getChannel())
    #     joysim = wpilib.simulation.JoystickSim(robot.lstick)
    #
    #     # Set the joystick value to something
    #     joysim.setY(0.5)
    #
    #     # Enable the robot, check to see if it was set
    #     control.step_timing(seconds=0.1, autonomous=False, enabled=True)
    #     assert 0.5 == pytest.approx(motorsim.getSpeed(), rel=0.01)
    #
    #     # change it, see if it's still set
    #     joysim.setY(0.2)
    #     control.step_timing(seconds=0.1, autonomous=False, enabled=True)
    #     assert 0.2 == pytest.approx(motorsim.getSpeed(), rel=0.01)
