import time
import copy
from threading import Lock

import rclpy
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Joy

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS


class WidowArm(InterbotixManipulatorXS):
    button_disable = 5
    button_sleep = 9

    axis_waist = 0
    axis_wrist_angle = 4
    axis_elbow = 1
    axis_shoulder_1 = 2
    axis_shoulder_2 = 5

    waist_multiplier = 0.02
    wrist_angle_multiplier = 0.02
    elbow_multipler = 0.02
    shoulder_multiplier = 0.02

    def __init__(self):
        InterbotixManipulatorXS.__init__(
            self,
            robot_model="wx250",
            robot_name="wx250",
            moving_time=1.0 / 30.0,
            accel_time=1.0 / 60.0,
            gripper_pressure=1.0,
            logging_level=LoggingSeverity.DEBUG,
        )

        self.rate = self.core.get_node().create_rate(30.0)

        self.joy_sub = self.core.get_node().create_subscription(
            Joy, "joy", self.joy_callback, 10
        )

        self.joy_msg = None
        self.joy_lock = Lock()

    def start(self):
        robot_startup()
        while rclpy.ok():
            self.controller()
            self.rate.sleep()

    def controller(self):
        self.arm.set_trajectory_time(0.05, 0.1)

        with self.joy_lock:
            if not self.joy_msg or self.joy_msg.buttons[self.button_disable]:
                return

            # if self.joy_msg.buttons[self.button_sleep]:
                # self.arm.set_trajectory_time(1.5, 0.75)
                # self.arm.go_to_home_pose(moving_time=1.5, accel_time=0.75, blocking=False)

            # waist_position = self.arm.get_single_joint_command("waist")
            # waist_speed = self.joy_msg.axes[self.axis_waist]

            # print(waist_position, waist_speed)

            # self.arm.set_single_joint_position(
            #     joint_name="waist",
            #     position=waist_position
            #     + waist_speed * self.waist_multiplier,
            #     blocking=False,
            # )


            wrist_angle_position = self.arm.get_single_joint_command("wrist_angle")
            wrist_angle_speed = self.joy_msg.axes[self.axis_wrist_angle]

            # print(wrist_angle_position, wrist_angle_speed)

            self.arm.set_single_joint_position(
                joint_name="wrist_angle",
                position=wrist_angle_position
                + wrist_angle_speed * self.wrist_angle_multiplier,
                blocking=False,
            )

            elbow_position = self.arm.get_single_joint_command("elbow")
            elbow_speed = self.joy_msg.axes[self.axis_elbow]

            # print(elbow_position, elbow_speed)

            self.arm.set_single_joint_position(
                joint_name="elbow",
                position=elbow_position + elbow_speed * self.elbow_multipler,
                blocking=False,
            )

            shoulder_position = self.arm.get_single_joint_command("shoulder")

            shoulder_speed = 0.0
            if self.joy_msg.axes[self.axis_shoulder_1] == 1.0:
                shoulder_speed = (1.0 - self.joy_msg.axes[self.axis_shoulder_2]) / 2.0
            elif self.joy_msg.axes[self.axis_shoulder_2] == 1.0:
                shoulder_speed = -(1.0 - self.joy_msg.axes[self.axis_shoulder_1]) / 2.0

            # print(shoulder_position, shoulder_speed)

            self.arm.set_single_joint_position(
                joint_name="shoulder",
                position=shoulder_position + shoulder_speed * self.shoulder_multiplier,
                blocking=False,
            )

    def joy_callback(self, joy_msg: Joy):
        with self.joy_lock:
            self.joy_msg = copy.deepcopy(joy_msg)
