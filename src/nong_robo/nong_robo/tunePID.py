import rclpy
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class CommandRobo(Node):
    def __init__(self):
        super().__init__("command_node")
        self.encoder = self.create_subscription(
            Twist,
            "genaral_topic",
            self.sub_encode_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.encoder

        self.sub_pid = self.create_subscription(
            Float32MultiArray,
            "pid_topic",
            self.sub_pid_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.sub_pid

        self.sent_command = self.create_publisher(
            Twist, "command_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_command_timer = self.create_timer(0.05, self.sent_command_callback)

        self.grab_mode = 0
        self.pre_grab = -1
        self.encode_value = 0

        self.spin_mode = 0
        self.pre_spin = -1
        self.lastTime = self.get_clock().now().to_msg().sec
        self.integral = 0
        self.lastError = 0

        self.box_pos = [0, 400, 800, 1200, 1600]
        self.select_Box = 0
        self.next_select_Box = 0
        self.setpoint = 0
        self.setpoint_adder = 0

        self.Kp = 0.0
        self.Ki = 0.0
        self.Kd = 0.0

    def PID(self, Kp, Ki, Kd, setpoint, current):
        error = setpoint - current
        self.integral += error
        derivative = error - self.lastError

        control_output = (Kp * error) + (Ki * self.integral) + (Kd * derivative)

        self.prev_error = error
        return control_output

    def sub_pid_callback(self, msg):
        self.Kp = msg.data[0]
        self.Ki = msg.data[1]
        self.Kd = msg.data[2]
        self.setpoint = msg.data[3]

    def sub_encode_callback(self, msg):
        self.encode_value = msg.linear.x

    def sent_command_callback(self):  # publisher drive topic
        msg = Twist()

        msg.angular.x = 29.0
        msg.angular.y = 160.0
        msg.angular.z = 20.0
        current = self.encode_value
        self.select_Box = self.next_select_Box
        if not (self.setpoint - 80 < current and self.setpoint + 80 > current):
            self.integral = 0
            self.lastError = 0
            value = self.PID(
                self.Kp,
                self.Ki,
                self.Kd,
                self.setpoint,
                current,
            )
            if abs(value) >= 100:
                value = 100.0 * value / abs(value)

            msg.linear.x = float(-1 * value)
        else:
            msg.linear.x = 0.0
        self.get_logger().info(
            f"{self.setpoint} {current} {self.Kp} {self.Ki} {self.Kd}"
        )

        self.sent_command.publish(msg)


def main():
    rclpy.init()

    sub = CommandRobo()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
