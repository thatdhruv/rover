"""SLAM Toolbox (online async) plus the lidar driver on hardware."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    slam_cfg = PathJoinSubstitution([FindPackageShare("rover_bringup"), "config", "slam.yaml"])

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_cfg, {"use_sim_time": ParameterValue(sim, value_type=bool)}],
        output="screen",
    )

    # Any 2D lidar with a ROS 2 driver works; sllidar_ros2 shown for an RPLIDAR A1/A2.
    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        parameters=[{"serial_port": "/dev/ttyUSB0", "frame_id": "lidar_link", "serial_baudrate": 115200}],
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([DeclareLaunchArgument("sim", default_value="false"), slam, lidar])
