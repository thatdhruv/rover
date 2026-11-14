"""Static checks that catch the mistakes that otherwise only show up on the robot."""
import pathlib

import yaml

CFG = pathlib.Path(__file__).resolve().parents[1] / "config"
URDF = pathlib.Path(__file__).resolve().parents[2] / "rover_description" / "urdf" / "rover.urdf.xacro"


def load(name):
    return yaml.safe_load((CFG / name).read_text())


def test_controller_geometry_matches_urdf():
    params = load("controllers.yaml")["diff_drive_controller"]["ros__parameters"]
    urdf = URDF.read_text()
    assert f'name="wheel_radius" value="{params["wheel_radius"]:.3f}"' in urdf
    assert f'name="track" value="{params["wheel_separation"]:.3f}"' in urdf


def test_firmware_geometry_matches_controller():
    params = load("controllers.yaml")["diff_drive_controller"]["ros__parameters"]
    src = (pathlib.Path(__file__).resolve().parents[4] / "firmware" / "app" / "src" / "control.c").read_text()
    assert f"WHEEL_RADIUS_M   {params['wheel_radius']:.3f}f" in src
    assert f"TRACK_WIDTH_M    {params['wheel_separation']:.3f}f" in src


def test_ekf_has_matrix_of_correct_size():
    ekf = load("ekf.yaml")["ekf_filter_node"]["ros__parameters"]
    assert len(ekf["process_noise_covariance"]) == 15 * 15
    assert len(ekf["odom0_config"]) == 15
    assert len(ekf["imu0_config"]) == 15


def test_diff_drive_does_not_fight_ekf_for_odom_tf():
    params = load("controllers.yaml")["diff_drive_controller"]["ros__parameters"]
    ekf = load("ekf.yaml")["ekf_filter_node"]["ros__parameters"]
    assert params["enable_odom_tf"] is False
    assert ekf["publish_tf"] is True
    assert ekf["odom0"] == "/diff_drive_controller/odom"


def test_slam_and_nav_share_frames():
    slam = load("slam.yaml")["slam_toolbox"]["ros__parameters"]
    nav = load("nav2.yaml")
    assert slam["base_frame"] == nav["bt_navigator"]["ros__parameters"]["robot_base_frame"]
    assert slam["map_frame"] == nav["bt_navigator"]["ros__parameters"]["global_frame"]
