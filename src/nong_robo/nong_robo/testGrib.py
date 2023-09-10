import rclpy
import numpy as np
import math

from rclpy.node import Node
from std_msgs.msg import Int32, String
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class CommandRobo(Node):
    def __init__(self):
        super().__init__("command_node")
        self.sent_command = self.create_publisher(
            Twist, "command_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_command_timer = self.create_timer(0.05, self.sent_command_callback)

        self.state_gribber = 0
        self.preTime = self.get_clock().now()

    def sent_command_callback(self):  # publisher drive topic
        msg = Twist()

        if self.state_gribber == 0:
            msg.angular.x = 29.0
            msg.angular.y = 75.0
            msg.angular.z = 105.0
            if self.get_clock().now() - self.preTime >= rclpy.duration.Duration(
                seconds=3
            ):
                self.state_gribber = 1
                self.preTime = self.get_clock().now()
        elif self.state_gribber == 1:
            msg.angular.x = 150.0
            msg.angular.y = 75.0
            msg.angular.z = 105.0
            if self.get_clock().now() - self.preTime >= rclpy.duration.Duration(
                seconds=3
            ):
                self.state_gribber = 2
                self.preTime = self.get_clock().now()
        elif self.state_gribber == 2:
            msg.angular.x = 150.0
            msg.angular.y = 160.0
            msg.angular.z = 20.0
            if self.get_clock().now() - self.preTime >= rclpy.duration.Duration(
                seconds=3
            ):
                self.state_gribber = 3
                self.preTime = self.get_clock().now()
        elif self.state_gribber == 3:
            msg.angular.x = 29.0
            msg.angular.y = 160.0
            msg.angular.z = 20.0
            if self.get_clock().now() - self.preTime >= rclpy.duration.Duration(
                seconds=3
            ):
                self.state_gribber = 4
                self.preTime = self.get_clock().now()
        else:
            self.state_gribber = 0
        msg.linear.x = 100.0

        self.sent_command.publish(msg)


def main():
    rclpy.init()

    sub = CommandRobo()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
