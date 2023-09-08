import os
import math
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ld = LaunchDescription()
    channel_type = LaunchConfiguration("channel_type", default="serial")
    serial_port = LaunchConfiguration("serial_port", default="/dev/ttyUSB0")
    serial_baudrate = LaunchConfiguration("serial_baudrate", default="256000")
    frame_id = LaunchConfiguration("frame_id", default="laser")
    inverted = LaunchConfiguration("inverted", default="false")
    angle_compensate = LaunchConfiguration("angle_compensate", default="true")
    scan_mode = LaunchConfiguration("scan_mode", default="Sensitivity")

    config = os.path.join(
        get_package_share_directory("nong_robo"), "config", "params.yaml"
    )

    node_microros = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyACM0"],
    )
    node_drive = Node(package="nong_robo", executable="drive_node", parameters=[config])
    node_command = Node(
        package="nong_robo", executable="command_node", parameters=[config]
    )
    node_lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        parameters=[
            {
                "channel_type": channel_type,
                "serial_port": serial_port,
                "serial_baudrate": serial_baudrate,
                "frame_id": frame_id,
                "inverted": inverted,
                "angle_compensate": angle_compensate,
                "scan_mode": scan_mode,
            }
        ],
        output="screen",
    )
    node_state = Node(package="nong_robo", executable="state_node")
    node_detect = Node(package="nong_robo", executable="detection_node")

    ld.add_action(node_microros)
    ld.add_action(node_drive)
    ld.add_action(node_lidar)
    ld.add_action(node_state)
    ld.add_action(node_detect)
    ld.add_action(node_command)

    return ld
