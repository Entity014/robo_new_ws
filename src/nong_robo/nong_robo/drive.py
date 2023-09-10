import rclpy
import numpy as np
import math
import time

from rclpy.node import Node
from std_msgs.msg import Float32, String, Int32, Int32MultiArray
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class DriveRobo(Node):
    def __init__(self):
        super().__init__("drive_node")
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

        self.direct_sub = self.create_subscription(
            String,
            "direction",
            self.sub_direct_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.direct_sub

        self.head_sub = self.create_subscription(
            Twist,
            "genaral_topic",
            self.sub_head_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.head_sub

        self.sent_drive = self.create_publisher(
            Twist, "drive_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_drive_timer = self.create_timer(0.05, self.sent_drive_callback)

        self.sent_theta_fun = self.create_publisher(
            Float32, "theta_topic", qos_profile=qos.qos_profile_system_default
        )
        self.sent_theta_fun_timer = self.create_timer(0.05, self.sent_theta_callback)

        self.declare_parameters("", [("speed_motor", None), ("path_auto", None)])
        self.speed = (
            self.get_parameter("speed_motor").get_parameter_value().double_value
        )

        self.theta_sent = 0.0
        self.state_overall = ""
        self.state_map = 0
        self.state_grib = 0
        self.heading = 0.0
        self.direct = ""
        self.motor_direct = 1
        self.ultra = 0.0

    def sub_state_overall_callback(self, msg):
        self.state_overall = msg.data

    def sub_state_map_callback(self, msg):
        self.state_map = msg.data

    def sub_state_grib_callback(self, msg):
        self.state_grib = msg.data

    def sub_head_callback(self, msg):
        self.heading = msg.angular.x
        self.ultra = msg.angular.y

    def sub_direct_callback(self, msg):
        self.direct = msg.data

    def sent_theta_callback(self):
        msg = Float32()
        msg.data = self.theta_sent
        self.sent_theta_fun.publish(msg)

    def sent_drive_callback(self):  # publisher drive topic
        msg = Twist()

        # motor
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        # Servo
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        servoFront = 0.0
        servoLeft = 0.0
        servoRight = 0.0
        theta_motor = 0
        theta_sevor = 0
        motorSpeed = 0

        if self.state_overall == "RUNNING":
            if self.state_map == 1:
                theta_motor = 0
                theta_sevor = 0
                motorSpeed = self.speed
            elif self.state_map == 2:
                theta_motor = math.radians(90)
                theta_sevor = math.radians(90)
                motorSpeed = self.speed
            elif self.state_map == 3:
                motorSpeed = 0
            elif self.state_map == 4:
                if self.direct == "LEFT":
                    self.motor_direct = -1
                else:
                    self.motor_direct = 1
                if self.state_grib == 1:
                    theta_motor = 0
                    theta_sevor = 0
                    motorSpeed = self.speed * self.motor_direct
                elif self.state_grib == 2:
                    theta_motor = math.radians(90)
                    theta_sevor = math.radians(90)
                    motorSpeed = self.speed
                elif self.state_grib == 3:
                    motorSpeed = 0
                elif self.state_grib == 5:
                    theta_motor = math.radians(90)
                    theta_sevor = math.radians(90)
                    motorSpeed = self.speed
                elif self.state_grib == 6:
                    motorSpeed = 0
        else:
            theta_motor = 0
            theta_sevor = 0
            motorSpeed = 0
            self.isFirst = True

        thetaDiff = lambda x: x - 180 if x > 180 else x + 180 if x < 0 else x
        servoFront = thetaDiff(88.70085 * math.sin(abs(theta_sevor)) + 81.97)
        servoLeft = thetaDiff(100.81482 * math.sin(abs(theta_sevor)) + 60.37)
        servoRight = thetaDiff(100.81446 * math.sin(abs(theta_sevor)) + 145.23)
        if abs(self.heading) >= 0.2 and (self.state_map == 1 or self.state_map == 4):
            servoRight += self.heading * 2.5
            if self.ultra >= 20:
                servoRight += 5
            elif self.ultra <= 10:
                servoRight += -5
        elif abs(self.heading) >= 0.2 and self.state_map == 2:
            servoFront += self.heading * 2
        motorFront = math.sin(theta_motor - math.pi / 4) * motorSpeed
        motorLeft = math.sin(theta_motor - math.pi / 4) * motorSpeed
        motorRight = math.cos(theta_motor - math.pi / 4) * motorSpeed

        msg.linear.x = float(round(motorFront))
        msg.linear.y = float(round(motorLeft))
        msg.linear.z = float(round(motorRight))
        msg.angular.x = float(round(servoFront))
        msg.angular.y = float(round(servoLeft))
        msg.angular.z = float(round(servoRight))

        self.sent_drive.publish(msg)


def main():
    rclpy.init()

    sub = DriveRobo()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
