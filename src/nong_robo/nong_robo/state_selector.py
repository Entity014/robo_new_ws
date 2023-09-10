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
        self.state_map = 0
        self.state_grib = 0
        self.state_gribber = 0

        self.pos_x = 0
        self.mission = np.zeros((5, 2), dtype=int)
        self.room = np.zeros((5, 1), dtype=int)
        self.shape = np.zeros((5, 1), dtype=int)
        self.number = []

        self.direct = ""
        self.distance = 0
        self.distance_move = 0
        self.dis_arr = [0.80, 1.60, 2.40, 3.20]
        self.dis_start = 0.2
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
        self.integral = 0
        self.lastError = 0

        self.set_new_setpoint = False
        self.sent_value_rotate = 0

    def PID(self, Kp, Ki, Kd, setpoint, current):
        error = setpoint - current
        self.integral += error
        derivative = error - self.lastError

        control_output = (Kp * error) + (Ki * self.integral) + (Kd * derivative)

        self.prev_error = error
        return control_output

    def sub_room_callback(self, msg):
        self.room = msg.data

    def sub_shape_callback(self, msg):
        self.shape = msg.data
        shape_np = np.array(self.shape)
        count = np.count_nonzero(shape_np == 0)
        if count != 5:
            for i, v in enumerate(self.shape):
                if v == 3:
                    self.shape[i] = 1
                elif v == 4:
                    self.shape[i] = 2
                elif v == 5:
                    self.shape[i] = 3
                elif v == 8:
                    self.shape[i] = 4
                elif v == 10:
                    self.shape[i] = 0
            self.mission[:, 0] = self.room
            self.mission[:, 1] = self.shape
            sorted_indices = np.argsort(self.mission[:, 0])[::-1]
            self.mission = self.mission[sorted_indices]

    def sub_number_callback(self, msg):
        self.number = msg.data

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
        if self.state_sent != "Reset" or self.state_sent != "Idle":
            if (self.state_map == 0 or self.state_map == 1) and not (
                ranges[960] <= 0.50 and ranges[1550] >= 2.0
            ):
                self.state_map = 1
            elif (self.state_map == 1 or self.state_map == 2) and not (
                ranges[1400] <= 1.1 and ranges[1550] <= 1.1
            ):
                self.state_map = 2
            elif not (
                len(self.room) == 5 and len(self.room) == 5 and len(self.room) == 5
            ) and (self.state_map == 2 or self.state_map == 3):
                self.state_map = 3
            else:
                self.state_map = 4

            if self.state_map == 4:
                if len(self.mission) == 5:
                    self.state_grib == 1

            if self.state_grib == 1:
                if self.isFirst:
                    if ranges[960] - 0.6 <= 0:
                        self.state_grib = 2
                else:
                    if ranges[960] - 0.8 <= 0:
                        self.state_grib = 2
            elif self.state_grib == 2:
                if self.ultra <= 5:
                    self.state_grib = 3
            elif self.state_grib == 3:
                self.next_box = self.mission[0][1]
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

                self.select_box = self.next_box

                if not (self.setpoint - 80 < current and self.setpoint + 80 > current):
                    self.integral = 0
                    self.lastError = 0
                    value = self.PID(
                        1e-15,
                        0,
                        1000,
                        self.setpoint,
                        current,
                    )
                    if abs(value) >= 100:
                        value = 100.0 * value / abs(value)

                    self.sent_value_rotate = float(-1 * value)
                else:
                    self.sent_value_rotate = 0.0
                    self.state_grib = 4
                self.lastTime = self.get_clock().now().to_msg().sec
            elif self.state_grib == 4:
                if self.state_gribber == 3:
                    self.state_grib == 5
            elif self.state_grib == 5:
                if self.ultra >= 50:
                    self.state_grib = 6
                    self.mission = self.mission[1:]
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
