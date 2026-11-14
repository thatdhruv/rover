"""SLAM Toolbox (online async) plus the lidar driver on hardware.

slam_toolbox is a lifecycle node, so its own launch file is used: it configures and activates the
node for us (autostart).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    slam_cfg = PathJoinSubstitution([FindPackageShare("rover_bringup"), "config", "slam.yaml"])

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "slam_params_file": slam_cfg,
            "use_sim_time": sim,
            "autostart": "true",
        }.items(),
    )

    # Any 2D lidar with a ROS 2 driver works; sllidar_ros2 shown for an RPLIDAR A1/A2.
    lidar = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        parameters=[{"serial_port": "/dev/ttyUSB0", "frame_id": "lidar_link", "serial_baudrate": 115200}],
        condition=UnlessCondition(sim),
    )

    return LaunchDescription([DeclareLaunchArgument("sim", default_value="false"), slam, lidar])
