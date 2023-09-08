import rclpy
import numpy as np

from rclpy.node import Node
from std_msgs.msg import Int32, String, Int32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Vector3
from rclpy import qos


class StateSelector(Node):
    def __init__(self):
        super().__init__("state_node")
        self.gen_sub = self.create_subscription(
            Twist,
            "genaral_topic",
            self.sub_gen_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.gen_sub

        self.motor_sub = self.create_subscription(
            Twist,
            "motor_topic",
            self.sub_motor_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.motor_sub

        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.sub_scan_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.scan_sub

        self.mission_room = self.create_subscription(
            Int32MultiArray,
            "mission/room",
            self.sub_room_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.mission_room

        self.mission_shape = self.create_subscription(
            Int32MultiArray,
            "mission/shape",
            self.sub_shape_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.mission_shape

        self.mission_number = self.create_subscription(
            Int32MultiArray,
            "mission/number",
            self.sub_number_callback,
            qos_profile=qos.qos_profile_sensor_data,
        )
        self.mission_number

        self.send_state_overall = self.create_publisher(
            String, "state/overall", qos_profile=qos.qos_profile_system_default
        )
        self.send_state_map = self.create_publisher(
            Int32, "state/map", qos_profile=qos.qos_profile_system_default
        )
        self.send_state_grib = self.create_publisher(
            Int32, "state/grib", qos_profile=qos.qos_profile_system_default
        )
        self.send_direction = self.create_publisher(
            String, "direction", qos_profile=qos.qos_profile_system_default
        )
        self.send_pwm_rotate = self.create_publisher(
            Int32, "pwm_rotate", qos_profile=qos.qos_profile_system_default
        )

        self.state_overall_timer = self.create_timer(
            0.05, self.sent_state_overall_callback
        )
        self.send_state_map_timer = self.create_timer(
            0.05, self.sent_state_map_callback
        )
        self.sent_state_grib_timer = self.create_timer(
            0.05, self.sent_state_grib_callback
        )
        self.sent_direction_timer = self.create_timer(
            0.05, self.sent_direction_callback
        )
        self.sent_pwm_rotate_timer = self.create_timer(
            0.05, self.sent_pwm_rotate_callback
        )
        self.state_limit = 0.0
        self.state_sent = ""
        self.state_map = 3
        self.state_grib = 0
        self.state_gribber = 0

        self.pos_x = 0
        self.mission = np.zeros((5, 3))
        self.room = []
        self.shape = []
        self.number = []

        self.direct = ""
        self.distance = 0
        self.distance_move = 0
        self.dis_arr = [0.80, 1.60, 2.40, 3.20]
        self.dis_start = 0.0
        self.isFirst = True
        self.select_room = 0
        self.next_room = 0

        self.ultra = 0

        self.encode = 0
        self.select_box = 0
        self.next_box = 0
        self.box_pos = [0, 400, 800, 1200, 1600]
        self.setpoint = 0
        self.setpoint_adder = 0
        self.lastTime = self.get_clock().now().to_msg().sec
        self.Integral = 0
        self.lastError = 0

        self.set_new_setpoint = False
        self.sent_value_rotate = 0

    def PID(self, Kp, Ki, Kd, min_power, setpoint, current):
        currentTime = self.get_clock().now().to_msg().sec
        if (currentTime - self.lastTime) == 0:
            deltha_time = 1e-2
        else:
            deltha_time = currentTime - self.lastTime
        error_value = setpoint - current
        self.Integral += error_value * deltha_time
        if abs(self.Integral) >= 255:
            self.Integral = 255 * (self.Integral / abs(self.Integral))
        Derivative = (error_value - self.lastError) / deltha_time
        self.lastError = error_value
        # self.get_logger().info(f"{self.Integral}, {setpoint} {current}")
        return (
            (error_value * Kp)
            + (self.Integral * Ki)
            + (Derivative * Kd)
            + (min_power * error_value / abs(error_value))
        )

    def sub_room_callback(self, msg):
        self.room = msg.data

    def sub_shape_callback(self, msg):
        self.shape = msg.data
        for i in self.shape:
            if i == 3:
                self.shape[i] = 1
            elif i == 4:
                self.shape[i] = 2
            elif i == 5:
                self.shape[i] = 3
            elif i == 8:
                self.shape[i] = 4
            elif i == 10:
                self.shape[i] = 0

    def sub_number_callback(self, msg):
        self.number = msg.data

        for i in range(len(self.number)):
            self.mission[i] = [self.room[i], self.shape[i], self.number[i]]

        sorted_indices = np.argsort(self.mission[:, 3])
        self.mission = self.mission[sorted_indices]

    def sub_motor_callback(self, msg):
        self.pos_x = msg.linear.x

    def sub_gen_callback(self, msg):
        self.encode_value = msg.linear.x
        self.state_limit = int(msg.linear.z)
        self.ultra = msg.angular.y

        if self.state_limit == 1:
            self.state_sent = "Running"
        elif self.state_limit == 2:
            self.state_sent = "Reset"
            self.state_map = 0
            self.state_grib = 0
        else:
            self.state_sent = "Idle"

    def sub_scan_callback(self, msg):
        ranges = msg.ranges
        if self.state_sent != "Reset":
            if (ranges[1380] <= 0.7 or ranges[1630] <= 0.7) and (
                self.state_map == 0 or self.state_map == 1
            ):
                self.state_map = 1
            elif (ranges[1400] >= 0.85 or ranges[1500] >= 0.75) and (
                self.state_map == 1 or self.state_map == 2
            ):
                self.state_map = 2
            elif (
                len(self.room) != 5 and len(self.room) != 5 and len(self.room) != 5
            ) and (self.state_map == 2 or self.state_map == 3):
                self.state_map = 3
            else:
                self.state_map = 4

            if self.state_map == 4:
                self.next_room = self.mission[0]
                if self.state_grib == 0:
                    if self.isFirst:
                        if len(self.number) - self.next_room != 0:
                            self.distance = -(
                                self.dis_start
                                + self.dis_arr[len(self.number) - self.next_room - 1]
                            )
                        else:
                            self.distance = -self.dis_start
                        self.direct = "LEFT"
                        self.isFirst = False
                    else:
                        if self.select_room > self.next_room:
                            self.distance = -(
                                self.dis_arr[self.select_room - self.next_room - 1]
                            )
                            self.direct = "LEFT"
                        else:
                            self.distance = self.dis_arr[
                                self.next_room - self.select_room - 1
                            ]
                            self.direct = "RIGHT"

                    self.distance_move = self.distance + self.pos_x
                    self.state_grib == 1

            if self.state_grib == 1:
                if self.pos_x - self.distance_move <= 0:
                    self.state_grib = 2
            elif self.state_grib == 2:
                if self.ultra <= 5:
                    self.state_grib = 3
            elif self.state_grib == 3:
                self.next_box = self.mission[1]
                current = self.encode_value
                if self.select_box < self.next_box:
                    if (
                        self.next_box - self.select_box
                        < len(self.box_pos) - self.next_box + self.select_box
                    ):
                        self.setpoint_adder = self.box_pos[
                            self.next_box - self.select_box
                        ]
                    else:
                        self.setpoint_adder = (
                            -1
                            * self.box_pos[
                                len(self.box_pos) - self.next_box + self.select_box
                            ]
                        )
                elif self.select_box == self.next_box:
                    self.set_new_setpoint = False
                    self.setpoint_adder = self.box_pos[self.next_box - self.select_box]
                else:
                    if (
                        self.select_box - self.next_box
                        > self.next_box - self.select_box + len(self.box_pos)
                    ):
                        self.setpoint_adder = self.box_pos[
                            self.next_box - self.select_box + len(self.box_pos)
                        ]
                    else:
                        self.setpoint_adder = (
                            -1 * self.box_pos[self.select_box - self.next_box]
                        )

                if self.select_box != self.next_box:
                    self.set_new_setpoint = True
                if self.set_new_setpoint:
                    self.setpoint = self.setpoint_adder + current

                self.select_Box = self.next_box

                if not (self.setpoint - 5 < current and self.setpoint + 5 > current):
                    self.Integral = 0
                    self.lastError = 0
                    value = self.PID(
                        0.01,  # 0.13
                        0.0,  # 0.13
                        0,
                        0,
                        self.setpoint,
                        current,
                    )
                    if abs(value) >= 255:
                        value = 255.0 * value / abs(value)

                    self.sent_value_rotate = -1 * value
                else:
                    self.sent_value_rotate = 0
                    self.state_grib = 4
                self.lastTime = self.get_clock().now().to_msg().sec
            elif self.state_grib == 4:
                if self.state_gribber == 2:
                    self.state_grib == 5
            elif self.state_grib == 5:
                if self.ultra >= 50:
                    self.state_grib = 6
            else:
                self.state_grib = 0

        # self.get_logger().info(f"{ranges[1400]} {ranges[1500]} {self.state_map}")

    def sent_state_map_callback(self):
        msg = Int32()
        msg.data = self.state_map
        self.send_state_map.publish(msg)

    def sent_state_overall_callback(self):
        msg = String()
        msg.data = self.state_sent.upper()
        self.send_state_overall.publish(msg)

    def sent_state_grib_callback(self):
        msg = Int32()
        msg.data = self.state_grib
        self.send_state_grib.publish(msg)

    def sent_direction_callback(self):
        msg = String()
        msg.data = self.direct
        self.send_direction.publish(msg)

    def sent_pwm_rotate_callback(self):
        msg = Int32()
        msg.data = self.sent_value_rotate
        self.send_pwm_rotate.publish(msg)


def main():
    rclpy.init()

    sub = StateSelector()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
