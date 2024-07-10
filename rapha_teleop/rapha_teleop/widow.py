import time
import copy
from threading import Lock

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.logging import LoggingSeverity
from sensor_msgs.msg import Joy

from interbotix_common_modules.common_robot.robot import robot_shutdown, robot_startup, InterbotixRobotNode
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS, InterbotixArmXSInterface
from interbotix_xs_modules.xs_robot.gripper import InterbotixGripperXSInterface
from interbotix_xs_modules.xs_robot.core import InterbotixRobotXSCore


class WidowArmNode(InterbotixRobotNode):
    button_disable = 5
    button_sleep = 9
    button_grasp = 11
    button_release = 12
    button_startup = 9
    button_shutdown = 8

    axis_waist = 0
    axis_wrist_angle = 4
    axis_wrist_rotate = 3
    axis_elbow = 1
    axis_shoulder_1 = 2
    axis_shoulder_2 = 5

    waist_multiplier = 0.06
    wrist_angle_multiplier = 0.06
    wrist_rotate_multiplier = -0.10
    elbow_multipler = 0.04
    shoulder_multiplier = 0.04

    joy_timeout = 0.5

    def __init__(self):
        super().__init__()

        self.core = InterbotixRobotXSCore(
            robot_model="wx250",
            robot_name="wx250",
            node=self,
        )
        self.arm = InterbotixArmXSInterface(
            core=self.core,
            robot_model="wx250",
            group_name="arm",
            moving_time=0.05,
            accel_time=0.1,
            iterative_update_fk=False
        )
        self.gripper = InterbotixGripperXSInterface(
            core=self.core,
            gripper_name="gripper",
            gripper_pressure=1.0,
            gripper_pressure_lower_limit=150,
            gripper_pressure_upper_limit=350,
        )

        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )

        self.joy_msg = None
        self.joy_stamp = None
        self.joy_lock = Lock()

        self.controller_timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.controller_timer = self.create_timer(1/10, self.controller, callback_group=self.controller_timer_cb_group)

        robot_startup(self)

    def controller(self):
        self.arm.set_trajectory_time(0.04, 0.1)

        with self.joy_lock:
            if (
                not self.joy_msg
                or self.joy_msg.buttons[self.button_disable]
                or time.time() - self.joy_stamp > self.joy_timeout
            ):
                return

            if self.joy_msg.axes[7] == -1.0:
                # self.arm.set_trajectory_time(1.5, 0.75)
                self.arm.go_to_sleep_pose(moving_time=1.5, accel_time=0.75, blocking=True)
                self.arm.set_trajectory_time(0.04, 0.1)
            elif self.joy_msg.axes[7] == 1.0:
                # self.arm.set_trajectory_time(1.5, 0.75)
                self.arm.go_to_home_pose(moving_time=1.5, accel_time=0.75, blocking=True)
                self.arm.set_trajectory_time(0.04, 0.1)

            waist_position = self.arm.get_single_joint_command("waist")
            waist_speed = self.joy_msg.axes[self.axis_waist]

            # print(waist_position, waist_speed)

            self.arm.set_single_joint_position(
                joint_name="waist",
                position=waist_position + waist_speed * self.waist_multiplier,
                blocking=False,
            )

            wrist_angle_position = self.arm.get_single_joint_command("wrist_angle")
            wrist_angle_speed = self.joy_msg.axes[self.axis_wrist_angle]

            # print(wrist_angle_position, wrist_angle_speed)

            self.arm.set_single_joint_position(
                joint_name="wrist_angle",
                position=wrist_angle_position
                + wrist_angle_speed * self.wrist_angle_multiplier,
                blocking=False,
            )

            wrist_rotate_position = self.arm.get_single_joint_command("wrist_rotate")
            wrist_rotate_speed = self.joy_msg.axes[self.axis_wrist_rotate]

            # print(wrist_rotate_position, wrist_rotate_speed)

            self.arm.set_single_joint_position(
                joint_name="wrist_rotate",
                position=wrist_rotate_position
                + wrist_rotate_speed * self.wrist_rotate_multiplier,
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

            if self.joy_msg.buttons[self.button_grasp]:
                self.gripper.grasp(delay=0.0)
            if self.joy_msg.buttons[self.button_release]:
                self.gripper.release(delay=0.0)

            if self.joy_msg.buttons[self.button_startup]:
                self.core.robot_torque_enable("group", "arm", True)
                self.arm.capture_joint_positions()
            if self.joy_msg.buttons[self.button_shutdown]:
                self.core.robot_torque_enable("group", "arm", False)
            
    def joy_callback(self, joy_msg: Joy):
        with self.joy_lock:
            self.joy_msg = copy.deepcopy(joy_msg)
            self.joy_stamp = time.time()
