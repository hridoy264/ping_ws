#!/usr/bin/env python3
"""Generate labelled physics diagnostics; never modify the canonical world.

The stripped worlds isolate collision cost. They are not robot acceptance
worlds and cannot validate feeding, containment, or moving-head performance.
"""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET


def build(output):
    package = Path(__file__).resolve().parents[1]
    world_path = package / 'worlds/physical_100.sdf'
    model_path = package / 'models/pingpong_r10/model.sdf'
    source_world = ET.parse(world_path)
    source_model = ET.parse(model_path).getroot().find('model')
    output.mkdir(parents=True, exist_ok=True)
    snapshot_model = output / 'models/pingpong_r10'
    shutil.copytree(model_path.parent, snapshot_model, dirs_exist_ok=True)
    manifest = {'purpose': 'Physics performance isolation only; no robot acceptance claim',
                'source_world_sha256': hashlib.sha256(world_path.read_bytes()).hexdigest(),
                'source_model_sha256': hashlib.sha256(model_path.read_bytes()).hexdigest(),
                'snapshot_resource_path': 'models',
                'snapshot_mesh_sha256': {
                    str(p.relative_to(output)): hashlib.sha256(p.read_bytes()).hexdigest()
                    for p in sorted(snapshot_model.glob('meshes/*')) if p.is_file()},
                'worlds': []}
    cases = [('full_1', 1, 'full', None, None), ('full_10', 10, 'full', None, None),
             ('full_100', 100, 'full', None, None), ('static_robot_100', 100, 'static', None, None),
             ('hopper_only_100', 100, 'hopper', None, None), ('floor_only_100', 100, 'floor', None, None),
             ('full_100_pgs', 100, 'full', None, 'pgs'),
             ('full_100_bullet', 100, 'full', 'bullet', None),
             ('full_100_bullet_pgs', 100, 'full', 'bullet', 'pgs')]
    for name, count, scope, detector, solver in cases:
        root = copy.deepcopy(source_world.getroot())
        world = root.find('world')
        world.set('name', name)
        world.find('physics/max_step_size').text = '0.001'
        if detector or solver:
            dart = ET.SubElement(world.find('physics'), 'dart')
            if detector:
                ET.SubElement(dart, 'collision_detector').text = detector
            if solver:
                ET.SubElement(ET.SubElement(dart, 'solver'), 'solver_type').text = solver
        robot = copy.deepcopy(source_model)
        include = next(i for i in world.findall('include')
                       if i.findtext('uri') == 'model://pingpong_r10')
        robot_pose = include.findtext('pose')
        world.remove(include)
        pose = robot.find('pose')
        if pose is None:
            pose = ET.SubElement(robot, 'pose')
        pose.text = robot_pose
        monitor = next(p for p in robot.findall('plugin')
                       if p.get('name') == 'pingpong::PhysicalBallMonitor')
        monitor.find('expected_ball_count').text = str(count)
        if scope != 'full':
            # Remove actuator controller overhead and lock robot bodies. This
            # intentionally prevents mechanism movement in diagnostic copies.
            for plugin in list(robot.findall('plugin')):
                if plugin is not monitor:
                    robot.remove(plugin)
            static = robot.find('static')
            if static is None:
                static = ET.SubElement(robot, 'static')
            static.text = 'true'
            for joint in list(robot.findall('joint')):
                robot.remove(joint)
        removed = []
        if scope in ('hopper', 'floor'):
            for link in robot.findall('link'):
                for collision in list(link.findall('collision')):
                    cname = collision.get('name', '')
                    keep = scope == 'hopper' and (
                        cname.startswith(('hopper_', 'funnel_slope_', 'loading_neck_wall_'))
                        or cname == 'feeder_floor')
                    if not keep:
                        removed.append(f'{link.get("name")}/{cname}')
                        link.remove(collision)
            # Pure physics diagnostic should avoid all mesh loading and visual
            # publishing costs, not just remove contact geometry.
            for link in robot.findall('link'):
                for visual in list(link.findall('visual')):
                    link.remove(visual)
        world.append(robot)
        for model in list(world.findall('model')):
            if model.get('name', '').startswith('inventory_ball_'):
                if int(model.get('name').rsplit('_', 1)[1]) > count:
                    world.remove(model)
        target = output / f'{name}.sdf'
        ET.indent(root)
        ET.ElementTree(root).write(target, encoding='utf-8', xml_declaration=True)
        manifest['worlds'].append({'name': name, 'file': target.name,
                                  'count': count, 'scope': scope,
                                  'time_step_s': .001,
                                  'dart_collision_detector': detector or 'default',
                                  'dart_constraint_solver': solver or 'default',
                                  'robot_collisions': len(robot.findall('.//collision')),
                                  'removed_robot_collisions': removed,
                                  'sha256': hashlib.sha256(target.read_bytes()).hexdigest()})
    (output / 'benchmark_worlds_manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    result = build(parser.parse_args().output)
    print(json.dumps([{key: w[key] for key in ('name', 'count', 'robot_collisions')}
                      for w in result['worlds']], indent=2))
