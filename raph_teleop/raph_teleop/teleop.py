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
from raph_interfaces.msg import SteeringMode
from raph_interfaces.srv import SetSteeringMode
from rclpy.node import Node
from rclpy.task import Future
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class RaphTeleop(Node):

    def __init__(self) -> None:
        super().__init__("raph_teleop", start_parameter_services=False)

        self.retrieve_parameters()

        self.set_steering_mode_client = self.create_client(
            SetSteeringMode, "controller/set_steering_mode"
        )
        self.calibrate_servos_client = self.create_client(
            Trigger, "controller/calibrate_servos"
        )

        self.current_steering_mode: int = SteeringMode.ACKERMANN
        self.target_steering_mode: int = SteeringMode.ACKERMANN

        self.is_changing_mode = False
        self.is_calibrating_servos = False
        self.deadman_pressed = False

        self.cmd_pub = self.create_publisher(
            AckermannDrive, "controller/cmd_ackermann", 10
        )
        self.joy_sub = self.create_subscription(Joy, "joy", self.joy_callback, 10)

    def retrieve_parameters(self) -> None:
        self.declare_parameter("acceleration", 2.0)
        self.declare_parameter("steering_angle_velocity", 2.0)
        self.declare_parameter("axis_speed", 1)
        self.declare_parameter("axis_steer", 3)
        self.declare_parameter("button_deadman", 5)
        self.declare_parameter("button_change_steering_mode", 1)
        self.declare_parameter("button_calibrate_servos", 0)
        self.declare_parameter("button_turbo", 4)
        self.declare_parameter("scale_speed", 1.0)
        self.declare_parameter("scale_steer", 1.1)
        self.declare_parameter("turbo_acceleration", 4.0)
        self.declare_parameter("turbo_scale_speed", 1.5)

        self.acceleration = (
            self.get_parameter("acceleration").get_parameter_value().double_value
        )
        self.steering_angle_velocity = (
            self.get_parameter("steering_angle_velocity")
            .get_parameter_value()
            .double_value
        )
        self.axis_speed = (
            self.get_parameter("axis_speed").get_parameter_value().integer_value
        )
        self.axis_steer = (
            self.get_parameter("axis_steer").get_parameter_value().integer_value
        )
        self.button_deadman = (
            self.get_parameter("button_deadman").get_parameter_value().integer_value
        )
        self.button_change_steering_mode = (
            self.get_parameter("button_change_steering_mode")
            .get_parameter_value()
            .integer_value
        )
        self.button_calibrate_servos = (
            self.get_parameter("button_calibrate_servos")
            .get_parameter_value()
            .integer_value
        )
        self.button_turbo = (
            self.get_parameter("button_turbo").get_parameter_value().integer_value
        )
        self.scale_speed = (
            self.get_parameter("scale_speed").get_parameter_value().double_value
        )
        self.scale_steer = (
            self.get_parameter("scale_steer").get_parameter_value().double_value
        )
        self.turbo_acceleration = (
            self.get_parameter("turbo_acceleration").get_parameter_value().double_value
        )
        self.turbo_scale_speed = (
            self.get_parameter("turbo_scale_speed").get_parameter_value().double_value
        )

    def change_steering_mode(self) -> None:
        if self.is_changing_mode:
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
        self.is_changing_mode = True

        future = self.set_steering_mode_client.call_async(req)

        future.add_done_callback(self.set_steering_mode_done_callback)

    def set_steering_mode_done_callback(self, future: Future) -> None:
        result: SetSteeringMode.Response = future.result()
        if not result.success:
            self.get_logger().error(
                "Failed to change steering mode: Service response negative: "
                f"{result.status_message}"
            )
        else:
            self.get_logger().info("Steering mode changed")
            self.current_steering_mode = self.target_steering_mode
        self.is_changing_mode = False

    def calibrate_servos(self) -> None:
        if self.is_calibrating_servos:
            return

        if not self.calibrate_servos_client.service_is_ready():
            self.get_logger().error("Failed to calibrate servos: Service not ready")
            return

        req = Trigger.Request()
        self.is_calibrating_servos = True

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
        self.is_calibrating_servos = False

    def joy_callback(self, data: Joy) -> None:
        if self.is_changing_mode or self.is_calibrating_servos:
            return

        if data.buttons[self.button_change_steering_mode] == 1:
            self.change_steering_mode()
            return

        if data.buttons[self.button_calibrate_servos] == 1:
            self.calibrate_servos()
            return

        if data.buttons[self.button_deadman] == 0 and not self.deadman_pressed:
            return

        turbo = data.buttons[self.button_turbo] == 1

        cmd = AckermannDrive()
        cmd.acceleration = self.turbo_acceleration if turbo else self.acceleration
        cmd.steering_angle_velocity = self.steering_angle_velocity

        if data.buttons[self.button_deadman] == 1:
            cmd.speed = data.axes[self.axis_speed] * (
                self.turbo_scale_speed if turbo else self.scale_speed
            )
            cmd.steering_angle = data.axes[self.axis_steer] * self.scale_steer

            self.deadman_pressed = True
        else:
            cmd.speed = 0.0
            cmd.steering_angle = 0.0
            self.deadman_pressed = False

        self.cmd_pub.publish(cmd)
