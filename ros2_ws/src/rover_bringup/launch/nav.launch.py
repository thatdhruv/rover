"""Nav2 navigation stack. Start robot.launch.py and slam.launch.py first.

In this Nav2 release navigation_launch.py starts the servers but not a lifecycle manager (that lives in
bringup_launch.py, which also wants its own map/AMCL/SLAM setup). SLAM Toolbox already provides
map -> odom here, so we run just the navigation servers plus a lifecycle manager for them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# keep in sync with nav2_bringup/launch/navigation_launch.py: get_lifecycle_nodes()
NAV_NODES = [
    "controller_server", "smoother_server", "planner_server", "route_server", "behavior_server",
    "velocity_smoother", "collision_monitor", "bt_navigator", "waypoint_follower",
    "docking_server", "following_server",
]


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
            # the keepout/speed-zone filters need mask maps and extra nodes we do not use
            "use_keepout_zones": "False",
            "use_speed_zones": "False",
        }.items(),
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": ParameterValue(sim, value_type=bool),
            "autostart": True,
            "node_names": NAV_NODES,
        }],
    )

    return LaunchDescription([DeclareLaunchArgument("sim", default_value="false"), navigation, lifecycle_manager])
