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

        self.sent_state_gribber = self.create_publisher(
            Int32, "state/gribber", qos_profile=qos.qos_profile_system_default
        )
        self.sent_state_gribber_timer = self.create_timer(
            0.05, self.sent_state_gribber_callback
        )

        self.state_overall_sub = self.create_subscription(
            String,
            "state/overall",
            self.sub_state_overall_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.state_overall_sub

        self.state_map_sub = self.create_subscription(
            Int32,
            "state/map",
            self.sub_state_map_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.state_map_sub

        self.state_grib_sub = self.create_subscription(
            Int32,
            "state/grib",
            self.sub_state_grib_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.state_grib_sub

        self.pwm_rotate_sub = self.create_subscription(
            Int32,
            "pwm_rotate",
            self.sub_pwm_rotate_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.pwm_rotate_sub

        self.state_gribber = 0
        self.state_overall = ""
        self.state_map = 0
        self.state_grib = 0
        self.pwm_rotate = 0

        self.set_new_setpoint = False
        self.setPID = False

    def sub_state_overall_callback(self, msg):
        self.state_overall = msg.data

    def sub_state_map_callback(self, msg):
        self.state_map = msg.data

    def sub_state_grib_callback(self, msg):
        self.state_grib = msg.data

    def sub_pwm_rotate_callback(self, msg):
        self.pwm_rotate = msg.data

    def sent_state_gribber_callback(self):
        msg = Int32()
        msg.data = self.state_gribber
        self.sent_state_gribber.publish(msg)

    def sent_command_callback(self):  # publisher drive topic
        msg = Twist()

        if self.state_overall == "RUNNING" and self.state_map == 4:
            if self.state_grib == 4:
                if self.state_gribber == 0:
                    msg.angular.x = 29.0
                    msg.angular.y = 75.0
                    msg.angular.z = 105.0
                elif self.state_gribber == 1:
                    msg.angular.x = 150.0
                    msg.angular.y = 75.0
                    msg.angular.z = 105.0
                elif self.state_gribber == 2:
                    msg.angular.x = 150.0
                    msg.angular.y = 160.0
                    msg.angular.z = 20.0
                else:
                    self.state_gribber = 0
            else:
                msg.angular.x = 29.0
                msg.angular.y = 160.0
                msg.angular.z = 20.0

        msg.linear.x = float(self.pwm_rotate)

        self.sent_command.publish(msg)


def main():
    rclpy.init()

    sub = CommandRobo()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
