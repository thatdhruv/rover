"""Gazebo simulation of the rover. Same controllers, EKF, SLAM and Nav2 as hardware.

    ros2 launch rover_gazebo sim.launch.py [headless:=true]
    ros2 launch rover_bringup slam.launch.py sim:=true
    ros2 launch rover_bringup nav.launch.py sim:=true
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    world = PathJoinSubstitution([FindPackageShare("rover_gazebo"), "worlds", "room.sdf"])

    gz_launch = PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
    gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
        condition=UnlessCondition(headless),
    )
    server_only = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch),
        launch_arguments={"gz_args": ["-r -s --headless-rendering ", world]}.items(),
        condition=IfCondition(headless),
    )

    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("rover_bringup"), "launch", "robot.launch.py"])
        ),
        launch_arguments={"sim": "true"}.items(),
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "rover", "-x", "0", "-y", "0", "-z", "0.05"],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        remappings=[("/imu", "/imu/data")],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        gui, server_only, robot, spawn, bridge,
    ])
