import rclpy
import numpy as np
import math
import time
import cv2
import pytesseract

from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, String, String, Int32
from rclpy import qos
from imutils.perspective import four_point_transform


class DetectionRobo(Node):
    def __init__(self):
        super().__init__("detection_node")
        self.sent_mission_room = self.create_publisher(
            Int32MultiArray, "mission/room", qos_profile=qos.qos_profile_system_default
        )
        self.sent_mission_room_timer = self.create_timer(
            0.05, self.sent_mission_room_callback
        )
        self.sent_mission_shape = self.create_publisher(
            Int32MultiArray, "mission/shape", qos_profile=qos.qos_profile_system_default
        )
        self.sent_mission_shape_timer = self.create_timer(
            0.05, self.sent_mission_shape_callback
        )
        self.sent_mission_number = self.create_publisher(
            Int32MultiArray,
            "mission/number",
            qos_profile=qos.qos_profile_system_default,
        )
        self.sent_mission_room_timer = self.create_timer(
            0.05, self.sent_mission_number_callback
        )

        self.cap = cv2.VideoCapture(2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        self.state_overall = ""
        self.state_map = 0
        self.room_arr = np.zeros((5, 1), dtype=int)
        self.binary_arr = np.ones((5, 3), dtype=int)
        self.shape_arr = np.zeros((5, 1), dtype=int)
        self.number_arr = np.zeros((5, 1), dtype=int)
        self.width, self.height = 720, 600
        self.lower_blue = np.array([139, 80, 0])
        self.upper_blue = np.array([179, 151, 50])

        self.isWarped = False
        self.document_contour = []

    def binaryList2Decimal(self, binary_matrix):
        decimal_values = []

        for binary_list in binary_matrix:
            decimal_value = 0
            power = len(binary_list) - 1

            for bit in binary_list:
                decimal_value += bit * (2**power)
                power -= 1

            decimal_values.append(decimal_value)

        return decimal_values

    def scan_detection(self, image):
        self.document_contour = np.array(
            [[0, 0], [self.width, 0], [self.width, self.height], [0, self.height]]
        )

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        threshold = cv2.adaptiveThreshold(
            gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 85, 6
        )

        contours, _ = cv2.findContours(
            threshold, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        max_area = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 15000:
                peri = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.015 * peri, True)
                if area > max_area and len(approx) == 4:
                    self.document_contour = approx
                    max_area = area

    def matrix(self, Loop_x, Loop_y, frame):
        x, y = 0, 0
        for i in range(Loop_x):
            for j in range(Loop_y):
                # cv2.rectangle(frame,(x + (i * int(WIDTH / 5)),y + (j * int(HEIGHT / 3))),(x + int(WIDTH / 5) + (i * int(WIDTH / 5)),y + int(HEIGHT / 3) + (j * int(HEIGHT / 3))),(255,0,0),2)
                new = frame[
                    y
                    + (j * int(self.height / Loop_y)) : y
                    + int(self.height / Loop_y)
                    + (j * int(self.height / Loop_y)),
                    x
                    + (i * int(self.width / Loop_x)) : x
                    + int(self.width / Loop_x)
                    + (i * int(self.width / Loop_x)),
                ]
                if j == 0:
                    gray = cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
                    _, threshold = cv2.threshold(gray, 70, 255, cv2.THRESH_BINARY_INV)
                    contours, _ = cv2.findContours(
                        threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                    )
                    for contour in contours:
                        area = cv2.contourArea(contour)
                        approx = cv2.approxPolyDP(
                            contour, 0.012 * cv2.arcLength(contour, True), True
                        )
                        bx, by, bw, bh = cv2.boundingRect(contour)
                        if area > 800:
                            if bx < new.shape[1] / 3.25:
                                self.binary_arr[j, 0] = 0
                            elif bx < 2 * new.shape[1] / 3.25:
                                self.binary_arr[j, 1] = 0
                            elif bx < new.shape[1]:
                                self.binary_arr[j, 2] = 0
                    value = self.binaryList2Decimal(self.binary_arr)
                    self.room_arr[j] = value[i]

                elif j == 1:
                    mask = cv2.inRange(new, self.lower_blue, self.upper_blue)
                    contours, _ = cv2.findContours(
                        mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE
                    )

                    for contour in contours:
                        area = cv2.contourArea(contour)
                        approx = cv2.approxPolyDP(
                            contour, 0.015 * cv2.arcLength(contour, True), True
                        )
                        if area >= 120:
                            if len(approx) == 3:
                                self.shape_arr[j, i] = 3
                            elif len(approx) == 4:
                                self.shape_arr[j, i] = 4
                            elif len(approx) >= 5 and len(approx) <= 7:
                                self.shape_arr[j, i] = 5
                            elif len(approx) > 7 and len(approx) < 9:
                                self.shape_arr[j, i] = 8
                            else:
                                self.shape_arr[j, i] = 10
                # else:
                #     gray_image = cv2.cvtColor(new, cv2.COLOR_BGR2GRAY)
                #     custom_config = r"--oem 3 --psm 6 outputbase digits"
                #     extracted_text = pytesseract.image_to_string(
                #         gray_image, config=custom_config
                #     )
                # self.get_logger().info(f"{extracted_text}")

    def sent_mission_room_callback(self):
        msg = Int32MultiArray()
        ref, frame = self.cap.read()
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        frame = cv2.flip(frame, 2)
        frame = cv2.flip(frame, 1)
        frame = cv2.resize(frame, (self.width, self.height))
        frame_copy = frame.copy()
        warped = np.zeros((480, 640, 3), dtype=np.uint8)
        self.scan_detection(frame_copy)

        if not self.isWarped:
            warped = four_point_transform(
                frame_copy, self.document_contour.reshape(4, 2)
            )
            warped = cv2.resize(warped, (self.width, self.height))
            if self.document_contour.tolist() != [
                [0, 0],
                [self.width, 0],
                [self.width, self.height],
                [0, self.height],
            ]:
                cv2.imshow("test", warped)
                self.isWarped = True

        if self.isWarped:
            self.matrix(5, 3, warped)
            # self.number_arr[0, i] = extracted_text

        cv2.imshow("test", frame)
        msg.data = self.room_arr.flatten().tolist()
        self.sent_mission_room.publish(msg)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            self.cap.release()
            cv2.destroyAllWindows()
            exit()

    def sent_mission_shape_callback(self):
        msg = Int32MultiArray()
        if self.state_overall == "RUNNING" and self.state_map == 3:
            msg.data = self.shape_arr.flatten().tolist()
            self.sent_mission_shape.publish(msg)

    def sent_mission_number_callback(self):
        msg = Int32MultiArray()
        if self.state_overall == "RUNNING" and self.state_map == 3:
            msg.data = self.number_arr.flatten().tolist()
            self.sent_mission_shape.publish(msg)


def main():
    rclpy.init()

    sub = DetectionRobo()
    rclpy.spin(sub)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
