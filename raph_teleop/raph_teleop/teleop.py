# Copyright 2024 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from ackermann_msgs.msg import AckermannDrive
from raph_interfaces.msg import DrivetrainState, SteeringMode, TurnInPlaceDrive
from raph_interfaces.srv import SetSteeringMode
from raph_teleop.raph_teleop_params import raph_teleop_params
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from rclpy.task import Future
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class RaphTeleop(Node):

    def __init__(self) -> None:
        super().__init__("raph_teleop")

        self.params_listener = raph_teleop_params.ParamListener(self)
        self.params = self.params_listener.get_params()
        self.params_listener.set_user_callback(
            lambda params: setattr(self, "params", params)
        )

        self.set_steering_mode_client = self.create_client(
            SetSteeringMode, "controller/set_steering_mode"
        )
        self.calibrate_servos_client = self.create_client(
            Trigger, "controller/calibrate_servos"
        )

        self.current_steering_mode: int = SteeringMode.ACKERMANN
        self.target_steering_mode: int = SteeringMode.ACKERMANN
        self.drivetrain_state: int = DrivetrainState.OPERATING_STATE_DISABLED

        self.prev_change_mode_pressed = False
        self.prev_calibrate_servos_pressed = False
        self.deadman_pressed = False

        self.cmd_ackermann_pub = self.create_publisher(
            AckermannDrive, "controller/cmd_ackermann", 10
        )
        self.cmd_turn_in_place_pub = self.create_publisher(
            TurnInPlaceDrive, "controller/cmd_turn_in_place", 10
        )
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.drivetrain_state_sub = self.create_subscription(
            DrivetrainState,
            "controller/drivetrain_state",
            self.drivetrain_state_callback,
            qos_profile,
        )

    def change_steering_mode(self) -> None:
        if (
            self.drivetrain_state
            == DrivetrainState.OPERATING_STATE_CHANGING_STEERING_MODE
        ):
            self.get_logger().warning("Steering mode change already in progress")
            return

        if not self.set_steering_mode_client.service_is_ready():
            self.get_logger().error("Failed to change steering mode: Service not ready")
            return

        if self.current_steering_mode == SteeringMode.TURN_IN_PLACE:
            self.target_steering_mode = SteeringMode.ACKERMANN
        else:
            self.target_steering_mode = SteeringMode.TURN_IN_PLACE

        req = SetSteeringMode.Request()
        req.steering_mode.data = self.target_steering_mode

        future = self.set_steering_mode_client.call_async(req)

        future.add_done_callback(self.set_steering_mode_done_callback)

    def set_steering_mode_done_callback(self, future: Future) -> None:
        result: SetSteeringMode.Response = future.result()
        if not result.success:
            self.get_logger().error(
                "Failed to change steering mode: Service response negative: "
                f"{result.status_message}"
            )
            self.target_steering_mode = self.current_steering_mode
        else:
            self.get_logger().info("Steering mode changed")
            self.current_steering_mode = self.target_steering_mode

    def calibrate_servos(self) -> None:
        if self.drivetrain_state == DrivetrainState.OPERATING_STATE_CALIBRATING_SERVOS:
            self.get_logger().warning("Controller is already calibrating servos")
            return

        if not self.calibrate_servos_client.service_is_ready():
            self.get_logger().error("Failed to calibrate servos: Service not ready")
            return

        req = Trigger.Request()

        future = self.calibrate_servos_client.call_async(req)

        future.add_done_callback(self.calibrate_servos_done_callback)

    def calibrate_servos_done_callback(self, future: Future) -> None:
        result: Trigger.Response = future.result()
        if not result.success:
            self.get_logger().error(
                f"Failed to calibrate servos: Service response negative: {result.message}"
            )
        else:
            self.get_logger().info("Servos calibrated")

    def drivetrain_state_callback(self, msg: DrivetrainState) -> None:
        self.current_steering_mode = msg.steering_mode.data
        self.drivetrain_state = msg.operating_state

    def joy_callback(self, data: Joy) -> None:
        if len(data.buttons) <= max(
            self.params.button_change_steering_mode,
            self.params.button_calibrate_servos,
            self.params.button_deadman,
            self.params.button_turbo,
        ):
            self.get_logger().error(
                "Received Joy message with insufficient buttons: "
                f"{len(data.buttons)} buttons received."
            )
            return

        if len(data.axes) <= max(
            self.params.axis_speed,
            self.params.axis_steer,
            self.params.axis_turn_in_place,
        ):
            self.get_logger().error(
                f"Received Joy message with insufficient axes: {len(data.axes)} axes received."
            )
            return

        change_mode_pressed = data.buttons[self.params.button_change_steering_mode] == 1
        calibrate_pressed = data.buttons[self.params.button_calibrate_servos] == 1

        drivetrain_busy = self.drivetrain_state in (
            DrivetrainState.OPERATING_STATE_CHANGING_STEERING_MODE,
            DrivetrainState.OPERATING_STATE_CALIBRATING_SERVOS,
        )

        should_calibrate_servos = (
            calibrate_pressed and not self.prev_calibrate_servos_pressed
        )
        should_change_mode = change_mode_pressed and not self.prev_change_mode_pressed

        self.prev_change_mode_pressed = change_mode_pressed
        self.prev_calibrate_servos_pressed = calibrate_pressed

        if drivetrain_busy:
            return

        if should_change_mode:
            self.change_steering_mode()
            return

        if should_calibrate_servos:
            self.calibrate_servos()
            return

        deadman_active = data.buttons[self.params.button_deadman] == 1

        if not deadman_active and not self.deadman_pressed:
            return

        turbo = data.buttons[self.params.button_turbo] == 1

        if self.current_steering_mode == SteeringMode.TURN_IN_PLACE:
            acceleration = (
                self.params.turbo_turn_in_place_acceleration
                if turbo
                else self.params.turn_in_place_acceleration
            )
            self.publish_turn_in_place_command(
                deadman_active, data, acceleration, turbo
            )
        else:
            acceleration = (
                self.params.turbo_ackermann_acceleration
                if turbo
                else self.params.ackermann_acceleration
            )
            self.publish_ackermann_command(deadman_active, data, acceleration, turbo)

    def publish_ackermann_command(
        self, deadman_active: bool, joy: Joy, acceleration: float, turbo: bool
    ) -> None:
        cmd = AckermannDrive()
        cmd.acceleration = acceleration
        cmd.jerk = self.params.ackermann_jerk
        cmd.steering_angle_velocity = self.params.steering_angle_velocity

        if deadman_active:
            speed_scale = (
                self.params.turbo_scale_speed if turbo else self.params.scale_speed
            )
            cmd.speed = joy.axes[self.params.axis_speed] * speed_scale
            cmd.steering_angle = (
                joy.axes[self.params.axis_steer] * self.params.scale_steer
            )
            self.deadman_pressed = True
        else:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
            self.deadman_pressed = False

        self.cmd_ackermann_pub.publish(cmd)

    def publish_turn_in_place_command(
        self, deadman_active: bool, joy: Joy, acceleration: float, turbo: bool
    ) -> None:
        cmd = TurnInPlaceDrive()
        cmd.acceleration = acceleration

        angular_scale = (
            self.params.turbo_scale_turn_in_place
            if turbo
            else self.params.scale_turn_in_place
        )
        if deadman_active:
            cmd.angular_velocity = (
                joy.axes[self.params.axis_turn_in_place] * angular_scale
            )
            self.deadman_pressed = True
        else:
            cmd.angular_velocity = 0.0
            self.deadman_pressed = False

        self.cmd_turn_in_place_pub.publish(cmd)
