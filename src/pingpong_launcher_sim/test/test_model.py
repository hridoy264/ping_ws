#!/usr/bin/env python3
"""Offline checks for errors that otherwise appear as unstable or inert Gazebo models.

Run with Python's unittest on any host; ROS, Gazebo, and numpy are not required.
This complements (and does not replace) test/runtime_smoke.py on Ubuntu 22.04.
"""

import math
from pathlib import Path
import re
import struct
import unittest
import xml.etree.ElementTree as ET


PACKAGE = Path(__file__).resolve().parents[1]
MODEL_PATH = PACKAGE / "models" / "pingpong_launcher" / "model.sdf"
ACTUATED = {
    "yaw_joint", "pitch_joint", "feeder_joint",
    "wheel_1_joint", "wheel_2_joint", "wheel_3_joint",
}


def numbers(text):
    return tuple(float(value) for value in text.split())


def rotation(rpy):
    roll, pitch, yaw = rpy
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


IDENTITY = (rotation((0, 0, 0)), (0, 0, 0))


def apply(transform, point):
    matrix, offset = transform
    return tuple(sum(matrix[i][j] * point[j] for j in range(3)) + offset[i]
                 for i in range(3))


def compose(parent, child):
    a, _ = parent
    b, offset = child
    return (tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3))
                        for j in range(3)) for i in range(3)), apply(parent, offset))


def inverse_apply(transform, point):
    matrix, offset = transform
    return tuple(sum(matrix[j][i] * (point[j] - offset[j]) for j in range(3))
                 for i in range(3))


def pose_transform(element):
    value = numbers(element.text) if element is not None else (0,) * 6
    if len(value) != 6 or not all(math.isfinite(v) for v in value):
        raise ValueError("Expected a finite six-value SDF pose")
    return rotation(value[3:]), value[:3]


def determinant(matrix):
    a, b, c = matrix
    return (a[0] * (b[1] * c[2] - b[2] * c[1])
            - a[1] * (b[0] * c[2] - b[2] * c[0])
            + a[2] * (b[0] * c[1] - b[1] * c[0]))


def stl_bounds(path):
    data = path.read_bytes()
    if len(data) >= 84 and len(data) == 84 + struct.unpack_from("<I", data, 80)[0] * 50:
        vertices = [struct.unpack_from("<3f", data, start + offset)
                    for start in range(84, len(data), 50) for offset in (12, 24, 36)]
    else:
        vertices = [numbers(match) for match in re.findall(
            r"\bvertex\s+([^\r\n]+)", data.decode("ascii"))]
    if not vertices:
        raise ValueError(f"No triangles in {path}")
    if not all(math.isfinite(value) for point in vertices for value in point):
        raise ValueError(f"Non-finite vertex in {path}")
    return tuple(max(point[i] for point in vertices) - min(point[i] for point in vertices)
                 for i in range(3))


class ModelContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sdf = ET.parse(MODEL_PATH).getroot()
        cls.model = cls.sdf.find("model")
        cls.links = {link.attrib["name"]: link for link in cls.model.findall("link")}
        cls.joints = {joint.attrib["name"]: joint for joint in cls.model.findall("joint")}

    def test_all_actuators_are_connected_to_world(self):
        self.assertEqual(set(self.joints) - {name for name, joint in self.joints.items()
                                         if joint.attrib["type"] == "fixed"}, ACTUATED)
        parents = {}
        for name, joint in self.joints.items():
            parent, child = joint.findtext("parent"), joint.findtext("child")
            self.assertIn(parent, set(self.links) | {"world"}, name)
            self.assertIn(child, self.links, name)
            self.assertNotIn(child, parents, f"Multiple parent joints for {child}")
            parents[child] = parent
            if name in ACTUATED:
                axis = numbers(joint.findtext("axis/xyz"))
                self.assertAlmostEqual(sum(v * v for v in axis), 1.0, places=6)
                self.assertNotEqual(joint.findtext("axis/limit/lower"),
                                    joint.findtext("axis/limit/upper"), name)
        for name in self.links:
            current, visited = name, set()
            while current != "world":
                self.assertNotIn(current, visited, f"Joint cycle through {name}")
                visited.add(current)
                self.assertIn(current, parents, f"Free-floating link: {current}")
                current = parents[current]
        self.assertNotEqual(self.model.findtext("static", "false").lower(), "true",
                            "A static model cannot articulate its joints")

    def test_positive_finite_physically_possible_link_inertias(self):
        for name, link in self.links.items():
            with self.subTest(link=name):
                mass = float(link.findtext("inertial/mass"))
                self.assertTrue(math.isfinite(mass) and mass > 0)
                inertia = link.find("inertial/inertia")
                values = {key: float(inertia.findtext(key))
                          for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
                self.assertTrue(all(math.isfinite(v) for v in values.values()))
                matrix = ((values["ixx"], values["ixy"], values["ixz"]),
                          (values["ixy"], values["iyy"], values["iyz"]),
                          (values["ixz"], values["iyz"], values["izz"]))
                self.assertGreater(matrix[0][0], 0)
                self.assertGreater(matrix[0][0] * matrix[1][1] - matrix[0][1] ** 2, 0)
                self.assertGreater(determinant(matrix), 0, "Inertia must be positive definite")
                # Every principal moment must be <= the sum of the other two.
                half_trace = sum(matrix[i][i] for i in range(3)) / 2
                shape = tuple(tuple((half_trace if i == j else 0) - matrix[i][j]
                                    for j in range(3)) for i in range(3))
                scale = max(abs(v) for row in matrix for v in row)
                for i in range(3):
                    self.assertGreaterEqual(shape[i][i], -1e-9 * scale)
                    for j in range(i + 1, 3):
                        self.assertGreaterEqual(shape[i][i] * shape[j][j] - shape[i][j] ** 2,
                                                -1e-9 * scale ** 2)
                self.assertGreaterEqual(determinant(shape), -1e-9 * scale ** 3)

    def test_mesh_assets_resolve_locally_at_meter_scale(self):
        meshes = self.model.findall(".//visual/geometry/mesh")
        self.assertTrue(meshes, "CAD visual meshes are missing")
        checked = set()
        for mesh in meshes:
            uri = mesh.findtext("uri")
            self.assertTrue(uri.startswith("model://"), uri)
            path = PACKAGE / "models" / uri.removeprefix("model://")
            self.assertTrue(path.is_file(), str(path))
            scale = numbers(mesh.findtext("scale", "1 1 1"))
            self.assertEqual(len(scale), 3)
            self.assertTrue(all(math.isfinite(v) and v > 0 for v in scale))
            if path not in checked:
                extents = stl_bounds(path)
                effective = max(extents[i] * scale[i] for i in range(3))
                self.assertGreater(effective, 0.0001, "Mesh collapsed during unit conversion")
                self.assertLess(effective, 2.0, "Millimeter CAD was not converted to meters")
                checked.add(path)

    def test_collisions_use_non_degenerate_physics_primitives(self):
        collisions = self.model.findall(".//link/collision")
        self.assertTrue(collisions)
        for collision in collisions:
            geometry = collision.find("geometry")
            self.assertEqual(len(geometry), 1)
            shape = geometry[0]
            self.assertIn(shape.tag, {"box", "cylinder", "sphere"},
                          "Detailed CAD meshes must not become convex hulls blocking the bore")
            dimensions = [float(value) for element in shape for value in element.text.split()]
            self.assertTrue(all(math.isfinite(v) and v > 0 for v in dimensions))

    def test_every_actuator_has_a_native_controller_and_state_feedback(self):
        plugins = self.model.findall("plugin")
        controlled = []
        published = set()
        topics = set()
        for plugin in plugins:
            name = plugin.attrib["name"]
            if "JointPositionController" in name or "JointController" in name:
                controlled.extend(element.text for element in plugin.findall("joint_name"))
                topic = plugin.findtext("topic")
                self.assertTrue(topic and topic.startswith("/pingpong/"), name)
                self.assertNotIn(topic, topics, f"Controllers share command topic {topic}")
                topics.add(topic)
            if "JointStatePublisher" in name:
                published.update(element.text for element in plugin.findall("joint_name"))
        self.assertEqual(set(controlled), ACTUATED)
        self.assertEqual(len(controlled), len(ACTUATED))
        self.assertTrue(ACTUATED.issubset(published), "Actuator feedback is incomplete")

    def test_muzzle_ball_does_not_intersect_launcher(self):
        plugins = [plugin for plugin in self.model.findall("plugin")
                   if plugin.find("muzzle_pose") is not None]
        self.assertEqual(len(plugins), 1, "Expected the calibrated launch plugin")
        plugin = plugins[0]
        head = plugin.findtext("head_link", "head_link")
        radius = float(plugin.findtext("ball_radius", "0.02"))
        cache = {"__model__": IDENTITY, "world": IDENTITY}

        def link_transform(name, visiting=()):
            if name in cache:
                return cache[name]
            self.assertNotIn(name, visiting, "SDF pose frame cycle")
            pose = self.links[name].find("pose")
            relative = pose.attrib.get("relative_to", "__model__") if pose is not None else "__model__"
            cache[name] = compose(link_transform(relative, visiting + (name,)), pose_transform(pose))
            return cache[name]

        muzzle = compose(link_transform(head), pose_transform(plugin.find("muzzle_pose")))
        for travel in (0.0, 0.025, 0.05, 0.1, 0.2):
            center = apply(muzzle, (travel, 0, 0))
            for name, link in self.links.items():
                for collision in link.findall("collision"):
                    pose = collision.find("pose")
                    relative = pose.attrib.get("relative_to", name) if pose is not None else name
                    frame = compose(link_transform(relative), pose_transform(pose))
                    point = inverse_apply(frame, center)
                    shape = collision.find("geometry")[0]
                    if shape.tag == "box":
                        half = [v / 2 for v in numbers(shape.findtext("size"))]
                        distance = math.sqrt(sum(max(abs(point[i]) - half[i], 0) ** 2
                                                 for i in range(3)))
                    elif shape.tag == "cylinder":
                        radial = math.hypot(point[0], point[1]) - float(shape.findtext("radius"))
                        axial = abs(point[2]) - float(shape.findtext("length")) / 2
                        distance = math.hypot(max(radial, 0), max(axial, 0))
                    elif shape.tag == "sphere":
                        distance = math.sqrt(sum(v * v for v in point)) - float(shape.findtext("radius"))
                    else:
                        self.fail(f"Unsupported collision shape {shape.tag}")
                    self.assertGreater(distance, radius + 0.0001,
                                       f"Ball path at {travel} m hits {name}/{collision.attrib['name']}")

    def test_world_has_required_physics_systems(self):
        worlds = list((PACKAGE / "worlds").glob("*.sdf"))
        self.assertTrue(worlds, "No runnable world included")
        for path in worlds:
            with self.subTest(world=path.name):
                world = ET.parse(path).getroot().find("world")
                names = {plugin.attrib["name"].split("::")[-1] for plugin in world.findall("plugin")}
                self.assertTrue({"Physics", "UserCommands", "SceneBroadcaster"}.issubset(names))
                physics = world.find("physics")
                self.assertGreater(float(physics.findtext("max_step_size")), 0)
                self.assertLessEqual(float(physics.findtext("max_step_size")), 0.002,
                                     "Fast 40 mm balls need a bounded integration timestep")


if __name__ == "__main__":
    unittest.main()
