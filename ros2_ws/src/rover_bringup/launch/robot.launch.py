"""Bring up the rover: robot_state_publisher, ros2_control, EKF.

On hardware (sim:=false) this also starts the micro-ROS agent and the controller
manager. In simulation Gazebo hosts the controller manager, so rover_gazebo's
launch file includes this one with sim:=true and only the shared parts start.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    serial_dev = LaunchConfiguration("serial_dev")
    controllers = PathJoinSubstitution([FindPackageShare("rover_bringup"), "config", "controllers.yaml"])
    ekf_cfg = PathJoinSubstitution([FindPackageShare("rover_bringup"), "config", "ekf.yaml"])
    xacro_file = PathJoinSubstitution([FindPackageShare("rover_description"), "urdf", "rover.urdf.xacro"])

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ", xacro_file,
            " sim:=", sim, " controllers_file:=", controllers,
        ]),
        value_type=str,
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": ParameterValue(sim, value_type=bool)}],
    )

    # Hardware only: Gazebo's plugin hosts the controller manager in simulation.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{"robot_description": robot_description}, controllers],
        output="screen",
        condition=UnlessCondition(sim),
    )

    micro_ros_agent = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        arguments=["serial", "--dev", serial_dev, "-b", "921600"],
        output="screen",
        condition=UnlessCondition(sim),
    )

    jsb = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
               "--param-file", controllers],
    )
    diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager",
               "--param-file", controllers],
    )
    # diff drive after joint states are up, so odometry starts from a valid state
    diff_after_jsb = RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[diff]))

    # The firmware publishes /rover/imu; the EKF config expects /imu/data (Gazebo's name).
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_cfg, {"use_sim_time": ParameterValue(sim, value_type=bool)}],
        remappings=[("/imu/data", "/rover/imu")],
        condition=UnlessCondition(sim),
    )
    ekf_sim = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_cfg, {"use_sim_time": True}],
        condition=IfCondition(sim),
    )

    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("serial_dev", default_value="/dev/ttyAMA0"),
        rsp, control_node, micro_ros_agent, jsb, diff_after_jsb, ekf, ekf_sim,
    ])
