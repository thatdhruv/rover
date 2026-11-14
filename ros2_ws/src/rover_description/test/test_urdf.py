"""The xacro must expand for both hardware and simulation variants."""
import pathlib
import subprocess
import xml.etree.ElementTree as ET

import pytest

XACRO = pathlib.Path(__file__).resolve().parents[1] / "urdf" / "rover.urdf.xacro"


def expand(*args):
    out = subprocess.run(
        ["xacro", str(XACRO), *args], check=True, capture_output=True, text=True
    ).stdout
    return ET.fromstring(out)


@pytest.mark.parametrize("sim", ["false", "true"])
def test_expands_with_two_wheel_joints(sim):
    root = expand(f"sim:={sim}", "controllers_file:=/tmp/c.yaml")
    joints = {j.get("name"): j.get("type") for j in root.findall("joint")}
    assert joints["left_wheel_joint"] == "continuous"
    assert joints["right_wheel_joint"] == "continuous"


def test_hardware_variant_uses_rover_plugin():
    root = expand("sim:=false")
    plugins = [p.text for p in root.iter("plugin")]
    assert "rover_hardware/RoverSystem" in plugins


def test_sim_variant_uses_gazebo_plugin():
    root = expand("sim:=true", "controllers_file:=/tmp/c.yaml")
    plugins = [p.text for p in root.iter("plugin")]
    assert "gz_ros2_control/GazeboSimSystem" in plugins


def test_track_width_matches_firmware():
    root = expand("sim:=false")
    ys = {
        j.get("name"): float(j.find("origin").get("xyz").split()[1])
        for j in root.findall("joint")
        if j.get("name", "").endswith("_wheel_joint")
    }
    assert ys["left_wheel_joint"] - ys["right_wheel_joint"] == pytest.approx(0.160)
