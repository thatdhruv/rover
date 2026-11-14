"""Nav2 navigation stack. Start robot.launch.py and slam.launch.py first."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    params = PathJoinSubstitution([FindPackageShare("rover_bringup"), "config", "nav2.yaml"])

    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "params_file": params,
            "use_sim_time": sim,
            "autostart": "true",
        }.items(),
    )
    return LaunchDescription([DeclareLaunchArgument("sim", default_value="false"), navigation])
