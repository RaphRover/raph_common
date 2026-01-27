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
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
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
        self.drivetrain_state: int = (
            DrivetrainState.OPERATING_STATE_DISABLED
        )

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

    def retrieve_parameters(self) -> None:
        self.declare_parameter("ackermann_acceleration", 2.0)
        self.declare_parameter("turn_in_place_acceleration", 2.0)
        self.declare_parameter("steering_angle_velocity", 2.0)
        self.declare_parameter("axis_speed", 1)
        self.declare_parameter("axis_steer", 3)
        self.declare_parameter("axis_turn_in_place", 0)
        self.declare_parameter("button_deadman", 5)
        self.declare_parameter("button_change_steering_mode", 1)
        self.declare_parameter("button_calibrate_servos", 0)
        self.declare_parameter("button_turbo", 4)
        self.declare_parameter("scale_speed", 1.0)
        self.declare_parameter("scale_steer", 1.1)
        self.declare_parameter("scale_turn_in_place", 1.1)
        self.declare_parameter("turbo_ackermann_acceleration", 4.0)
        self.declare_parameter("turbo_turn_in_place_acceleration", 4.0)
        self.declare_parameter("turbo_scale_speed", 1.5)
        self.declare_parameter("turbo_scale_turn_in_place", 1.5)

        self.ackermann_acceleration = (
            self.get_parameter("ackermann_acceleration")
            .get_parameter_value()
            .double_value
        )
        self.turn_in_place_acceleration = (
            self.get_parameter("turn_in_place_acceleration")
            .get_parameter_value()
            .double_value
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
        self.axis_turn_in_place = (
            self.get_parameter("axis_turn_in_place")
            .get_parameter_value()
            .integer_value
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
        self.scale_turn_in_place = (
            self.get_parameter("scale_turn_in_place")
            .get_parameter_value()
            .double_value
        )
        self.turbo_ackermann_acceleration = (
            self.get_parameter("turbo_ackermann_acceleration")
            .get_parameter_value()
            .double_value
        )
        self.turbo_turn_in_place_acceleration = (
            self.get_parameter("turbo_turn_in_place_acceleration")
            .get_parameter_value()
            .double_value
        )
        self.turbo_scale_speed = (
            self.get_parameter("turbo_scale_speed").get_parameter_value().double_value
        )
        self.turbo_scale_turn_in_place = (
            self.get_parameter("turbo_scale_turn_in_place")
            .get_parameter_value()
            .double_value
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
        if (
            self.drivetrain_state
            == DrivetrainState.OPERATING_STATE_CALIBRATING_SERVOS
        ):
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
        change_mode_pressed = data.buttons[self.button_change_steering_mode] == 1
        calibrate_pressed = data.buttons[self.button_calibrate_servos] == 1

        drivetrain_busy = self.drivetrain_state in (
            DrivetrainState.OPERATING_STATE_CHANGING_STEERING_MODE,
            DrivetrainState.OPERATING_STATE_CALIBRATING_SERVOS,
        )

        should_calibrate_servos = calibrate_pressed and not self.prev_calibrate_servos_pressed
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

        deadman_active = data.buttons[self.button_deadman] == 1

        if not deadman_active and not self.deadman_pressed:
            return

        turbo = data.buttons[self.button_turbo] == 1

        if self.current_steering_mode == SteeringMode.TURN_IN_PLACE:
            acceleration = (
                self.turbo_turn_in_place_acceleration
                if turbo
                else self.turn_in_place_acceleration
            )
            self.publish_turn_in_place_command(
                deadman_active, data, acceleration, turbo
            )
        else:
            acceleration = (
                self.turbo_ackermann_acceleration
                if turbo
                else self.ackermann_acceleration
            )
            self.publish_ackermann_command(deadman_active, data, acceleration, turbo)

    def publish_ackermann_command(
        self, deadman_active: bool, joy: Joy, acceleration: float, turbo: bool
    ) -> None:
        cmd = AckermannDrive()
        cmd.acceleration = acceleration
        cmd.steering_angle_velocity = self.steering_angle_velocity

        if deadman_active:
            speed_scale = self.turbo_scale_speed if turbo else self.scale_speed
            cmd.speed = joy.axes[self.axis_speed] * speed_scale
            cmd.steering_angle = joy.axes[self.axis_steer] * self.scale_steer
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
            self.turbo_scale_turn_in_place if turbo else self.scale_turn_in_place
        )
        if deadman_active:
            cmd.angular_velocity = joy.axes[self.axis_turn_in_place] * angular_scale
            self.deadman_pressed = True
        else:
            cmd.angular_velocity = 0.0
            self.deadman_pressed = False

        self.cmd_turn_in_place_pub.publish(cmd)
