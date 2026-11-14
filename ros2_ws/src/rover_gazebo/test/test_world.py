"""The world file must be well-formed SDF and contain the plugins sensors need."""
import pathlib
import xml.etree.ElementTree as ET

WORLD = pathlib.Path(__file__).resolve().parents[1] / "worlds" / "room.sdf"


def test_world_parses_and_has_required_systems():
    root = ET.parse(WORLD).getroot()
    plugins = {p.get("filename") for p in root.iter("plugin")}
    assert {"gz-sim-physics-system", "gz-sim-sensors-system", "gz-sim-imu-system"} <= plugins


def test_room_has_obstacles_for_slam():
    root = ET.parse(WORLD).getroot()
    models = {m.get("name") for m in root.iter("model")}
    assert {"walls", "box_a", "pillar", "box_b"} <= models
