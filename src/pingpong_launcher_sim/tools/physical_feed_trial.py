#!/usr/bin/env python3
"""Observe persistent physical balls while priming the R10 feed route.

This starts one isolated Fortress world, settles with motors stopped, then
commands only the existing feeder joint velocity. Launch wheels remain stopped.
No ball entities, body poses, forces, impulses, or velocities are commanded.
Native poses and joint states, rather than an abstract shot counter, establish
route occupancy and arrival. A completed trial without head arrival fails.
The default inventory target is 100. Explicit smaller --count values describe
staged diagnostics and cannot establish a 100-ball acceptance result.
"""

import argparse
from collections import Counter, deque
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import queue
import re
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

from physical_inventory_smoke import InventoryAudit, decode_topic_line, finite_number


PACKAGE = Path(__file__).resolve().parents[1]
ROUTE = ('pan_outlet', 'lower_bend', 'riser', 'yaw_elbow', 'head_elbow', 'head')
STAGES = ('bin', 'feeder', *ROUTE, 'muzzle', 'outside')
JOINTS = ('yaw_joint', 'pitch_joint', 'feeder_joint',
          'wheel_1_joint', 'wheel_2_joint', 'wheel_3_joint')
REQUIRED_TRANSIT = ('feeder', 'lower_bend', 'riser', 'yaw_elbow', 'head_elbow', 'head')
TOKEN = re.compile(r'\s*("(?:\\.|[^"\\])*"|[{}:]|[^\s{}:]+)')


def priming_budget(config, feeder_speed, feeding_duration):
    """Ideal geometry estimate only; pocket filling and friction are unproved.

    A push column cannot transmit tension. Insufficient inventory cannot cold
    prime this rising path, however long the feeder rotates. The last balls
    also remain in the path after the hopper empties.
    """
    lengths = {name: sum(math.dist(a, b) for a, b in
                        zip(route['centres'], route['centres'][1:]))
               for name, route in config['routes'].items()}
    diameter = 2 * config['ball_radius']
    period = math.tau / (6 * feeder_speed)
    to_head = sum(lengths[name] for name in ROUTE if name != 'head')
    to_muzzle = sum(lengths.values())
    # First loaded pocket travels a quarter turn from the rear loading port
    # to the tangential outlet. Later balls arrive one pocket pitch apart.
    first_delivery = math.pi / (2 * feeder_speed)
    head_count = math.ceil(to_head / diameter) + 1
    muzzle_count = math.ceil(to_muzzle / diameter) + 1
    head_time = first_delivery + (head_count - 1) * period
    muzzle_time = first_delivery + (muzzle_count - 1) * period
    return {'scope': 'Ideal packed-column lower-bound planning estimate, not a transit result.',
            'path_to_head_inlet_m': to_head, 'path_to_muzzle_m': to_muzzle,
            'nominal_pocket_period_s': period, 'ideal_first_outlet_delivery_s': first_delivery,
            'estimated_balls_to_reach_head_inlet': head_count,
            'estimated_balls_to_reach_muzzle': muzzle_count,
            'ideal_head_inlet_time_s': head_time, 'ideal_muzzle_time_s': muzzle_time,
            'available_feeding_time_s': feeding_duration,
            'duration_covers_ideal_head_arrival': feeding_duration >= head_time,
            'inventory_can_form_estimated_head_column': len(config['expected_names']) >= head_count,
            'remaining_path_inventory_needs_physical_drain': True}


def subtract(a, b):
    return tuple(x-y for x, y in zip(a, b))


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def rotate_z(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c*p[0]-s*p[1], s*p[0]+c*p[1], p[2])


def rotate_y(p, angle):
    c, s = math.cos(angle), math.sin(angle)
    return (c*p[0]+s*p[2], p[1], -s*p[0]+c*p[2])


def about(p, origin, rotation, angle):
    return tuple(a+b for a, b in zip(origin, rotation(subtract(p, origin), angle)))


def inverse_quaternion(p, q):
    # Conjugate rotation: v' = v + 2*w*(u x v) + 2*(u x (u x v)).
    w, x, y, z = q
    u = (-x, -y, -z)
    uv = (u[1]*p[2]-u[2]*p[1], u[2]*p[0]-u[0]*p[2], u[0]*p[1]-u[1]*p[0])
    uuv = (u[1]*uv[2]-u[2]*uv[1], u[2]*uv[0]-u[0]*uv[2], u[0]*uv[1]-u[1]*uv[0])
    return tuple(v+2*w*a+2*b for v, a, b in zip(p, uv, uuv))


def parse_proto(text):
    """Parse protobuf text emitted by ign topic; retain repeated fields."""
    tokens = TOKEN.findall(text)
    index = 0

    def fields(nested=False):
        nonlocal index
        result = {}
        while index < len(tokens):
            key = tokens[index]
            index += 1
            if key == '}':
                if not nested:
                    raise ValueError('Unexpected protobuf closing brace')
                return result
            if not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', key) or index >= len(tokens):
                raise ValueError('Malformed protobuf field')
            separator = tokens[index]
            index += 1
            if separator == '{':
                value = fields(True)
            elif separator == ':' and index < len(tokens):
                value = tokens[index]
                index += 1
                if value.startswith('"'):
                    value = json.loads(value)
                elif value in ('true', 'false'):
                    value = value == 'true'
                else:
                    try:
                        value = float(value) if any(c in value for c in '.eE') else int(value)
                    except ValueError:
                        pass  # Protobuf enum; not used for joint measurements.
            else:
                raise ValueError('Malformed protobuf value')
            result.setdefault(key, []).append(value)
        if nested:
            raise ValueError('Incomplete protobuf message')
        return result

    return fields()


def one(obj, key, default=None):
    values = obj.get(key)
    if not values:
        return default
    if len(values) != 1:
        raise ValueError(f'Duplicate singular protobuf field {key}')
    return values[0]


def decode_joint_message(text, model_name):
    message = parse_proto(text)
    if one(message, 'name') != model_name:
        raise ValueError('Joint state topic belongs to another model')
    stamp = one(one(message, 'header', {}), 'stamp', {})
    now = finite_number(one(stamp, 'sec', 0), 'joint seconds') + \
        finite_number(one(stamp, 'nsec', 0), 'joint nanoseconds') * 1e-9
    measurements = {}
    for joint in message.get('joint', []):
        name = one(joint, 'name')
        if name not in JOINTS:
            continue
        axis = one(joint, 'axis1')
        if axis is None:
            return None  # Physics has not populated the initial state yet.
        if name in measurements:
            raise ValueError('Duplicate measured joint')
        measurements[name] = {
            'entity': one(joint, 'id', 0),
            'position': finite_number(one(axis, 'position', 0), name+'.position'),
            'velocity': finite_number(one(axis, 'velocity', 0), name+'.velocity'),
        }
    if set(measurements) != set(JOINTS):
        return None
    pose = one(message, 'pose')
    if pose is None:
        raise ValueError('Native model pose is unavailable')
    position, orientation = one(pose, 'position', {}), one(pose, 'orientation', {})
    xyz = tuple(finite_number(one(position, a, 0), 'model.'+a) for a in ('x', 'y', 'z'))
    quaternion = tuple(finite_number(one(orientation, a, 0), 'model.q'+a)
                       for a in ('w', 'x', 'y', 'z'))
    if abs(dot(quaternion, quaternion)-1) > 1e-3:
        raise ValueError('Native model orientation is not a unit quaternion')
    if any(j['entity'] <= 0 for j in measurements.values()):
        raise ValueError('Native joint entity ID is invalid')
    return {'sim_time': now, 'model_position': xyz, 'model_orientation': quaternion,
            'joints': measurements}


def read_inventory(pipe, messages):
    try:
        for line in iter(pipe.readline, ''):
            payload = decode_topic_line(line)
            if payload is not None:
                messages.put(('inventory', payload))
    except Exception as error:
        messages.put(('error', 'Inventory subscriber: '+str(error)))
    finally:
        messages.put(('closed', 'inventory'))


def read_joints(pipe, messages, model_name):
    lines = []
    last_sent = None
    try:
        for line in iter(pipe.readline, ''):
            if line.strip():
                lines.append(line)
            elif lines:
                text = ''.join(lines)
                lines.clear()
                if not text.lstrip().startswith('header'):
                    continue
                # JointStatePublisher can emit every physics iteration. Keep
                # 50 Hz simulation-time feedback without queuing thousands of
                # redundant states while the CLI publishes an actuator command.
                stamp = re.search(r'stamp\s*\{([^{}]*)\}', text)
                if stamp is None:
                    raise ValueError('Native joint message lacks a timestamp')
                parts = parse_proto(stamp.group(1))
                now = float(one(parts, 'sec', 0))+float(one(parts, 'nsec', 0))*1e-9
                if last_sent is not None and 0 <= now-last_sent < .02:
                    continue
                payload = decode_joint_message(text, model_name)
                if payload is not None:
                    messages.put(('joint', payload))
                    last_sent = now
    except Exception as error:
        messages.put(('error', 'Joint subscriber: '+str(error)))
    finally:
        messages.put(('closed', 'joint'))


def stop_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=0.5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=0.5)


def cli(command, env, deadline):
    remaining = deadline-time.monotonic()
    if remaining <= 0:
        raise TimeoutError('Trial wall-clock deadline reached')
    process = subprocess.Popen(command, env=env, stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        output = process.communicate(timeout=min(5.0, remaining))[0]
    except subprocess.TimeoutExpired:
        stop_process(process)
        raise TimeoutError('Transport command timed out: '+command[1])
    if process.returncode:
        raise RuntimeError('Transport command failed: '+output.strip())
    return output


def publish(topic, value, env, deadline):
    cli(['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Double', '-p',
         f'data: {value:.17g}'], env, deadline)


def load_config(world_path, model_dir, expected_count=100):
    world = ET.parse(world_path).getroot().find('world')
    model_path = model_dir/'model.sdf'
    model = ET.parse(model_path).getroot().find('model')
    manifest_path = model_dir/'generation_manifest.json'
    manifest = json.loads(manifest_path.read_text())
    monitor = next((p for p in model.findall('plugin')
                    if p.get('name') == 'pingpong::PhysicalBallMonitor'), None)
    if monitor is None:
        raise ValueError('PhysicalBallMonitor is missing')
    prefix = monitor.findtext('ball_name_prefix', 'inventory_ball_')
    names = {f'{prefix}{i:03d}' for i in range(1, expected_count+1)}
    balls = [m for m in world.findall('model') if m.get('name', '').startswith(prefix)]
    if len(balls) != expected_count or {b.get('name') for b in balls} != names:
        raise ValueError(f'World must contain exactly the {expected_count} expected persistent ball models')
    ball_radius = float(monitor.findtext('ball_radius', '.020'))
    if not prefix or not math.isfinite(ball_radius) or ball_radius <= 0:
        raise ValueError('A nonempty ball prefix and positive radius are required')
    for ball in balls:
        link = ball.find("link[@name='ball_link']")
        radii = [float(c.findtext('geometry/sphere/radius', 'nan'))
                 for c in ([] if link is None else link.findall('collision'))]
        mass = float(link.findtext('inertial/mass', 'nan')) if link is not None else math.nan
        if ball.findtext('static', 'false').lower() == 'true' or len(radii) != 1 or \
                abs(radii[0]-ball_radius) > 1e-9 or not math.isfinite(mass) or mass <= 0:
            raise ValueError('Inventory must contain dynamic physical spheres of the configured radius')
    includes = [i for i in world.findall('include')
                if i.findtext('uri') == 'model://'+model.get('name')]
    if len(includes) != 1 or int(monitor.findtext('expected_ball_count', '100')) != expected_count:
        raise ValueError(f'Expected one robot include and a monitor configured for {expected_count} balls')
    joint_axis_signs = {}
    for name, link_name, datum, axis in (
            ('yaw_joint', 'yaw_link', 'yaw_origin_m', (0, 0, 1)),
            ('pitch_joint', 'head_link', 'pitch_origin_m', (0, 1, 0)),
            ('feeder_joint', 'feeder_link', 'feeder_origin_m', (0, 0, 1))):
        joint = model.find(f"joint[@name='{name}']")
        link = model.find(f"link[@name='{link_name}']")
        if joint is None or link is None:
            raise ValueError('Required physical joint/link missing')
        xyz = tuple(float(v) for v in joint.findtext('axis/xyz', '').split())
        pose = tuple(float(v) for v in link.findtext('pose', '').split())
        reverse_axis = tuple(-v for v in axis)
        if xyz not in (axis, reverse_axis) or len(pose) != 6 or any(abs(v) > 1e-9 for v in pose[3:]) or \
                math.dist(pose[:3], manifest[datum]) > 1e-9:
            raise ValueError('Manifest datums or joint axes disagree with the generated SDF')
        joint_axis_signs[name] = 1 if xyz == axis else -1
    for root in (world, model):
        for plugin in root.iter('plugin'):
            if 'BallLauncher' in plugin.get('name', '') or \
                    'pingpong_ball_launcher' in plugin.get('filename', ''):
                raise ValueError('Legacy spawn/launch plugin is prohibited in the physical trial')
    controllers = {}
    for plugin in model.findall('plugin'):
        name = plugin.findtext('joint_name')
        if name in JOINTS and 'Controller' in plugin.get('name', ''):
            topic = plugin.findtext('topic')
            if not topic or name in controllers:
                raise ValueError('Missing or duplicate actuator command interface')
            controllers[name] = topic
    if set(controllers) != set(JOINTS):
        raise ValueError('All six existing physical mechanism controllers are required')
    joint_publisher = next((p for p in model.findall('plugin')
                            if p.get('name', '').endswith('::JointStatePublisher')), None)
    if joint_publisher is None or not joint_publisher.findtext('topic'):
        raise ValueError('Native joint-state topic is required')
    routes = manifest.get('feed_route_model_m')
    if routes is None or set(routes) != set(ROUTE):
        raise ValueError('Geometry manifest needs all six explicit feed_route_model_m centerlines')
    for name, route in routes.items():
        if route.get('coordinate_frame') != 'neutral_model' or \
                route.get('frame') not in ('base', 'yaw', 'head') or \
                len(route.get('centres', [])) < 2 or \
                not math.isfinite(route.get('inner_radius_m', math.nan)) or \
                route['inner_radius_m'] <= ball_radius:
            raise ValueError('Invalid geometry manifest route '+name)
        for p in route['centres']:
            if len(p) != 3 or not all(math.isfinite(v) for v in p):
                raise ValueError('Non-finite route centre')
    return {'world_name': world.get('name'), 'model_name': model.get('name'),
            'controllers': controllers, 'joint_topic': joint_publisher.findtext('topic'),
            'inventory_topic': monitor.findtext('inventory_topic', '/pingpong/physical_ball_inventory'),
            'expected_names': sorted(names), 'manifest': manifest, 'routes': routes,
            'joint_axis_signs': joint_axis_signs,
            'ball_radius': ball_radius, 'aperture_radius': float(monitor.findtext('aperture_radius')),
            'muzzle_plane_x': float(monitor.findtext('muzzle_plane_x', '0')),
            'input_hashes': {str(p): hashlib.sha256(p.read_bytes()).hexdigest()
                             for p in (world_path, model_path, manifest_path)}}


class Geometry:
    def __init__(self, config):
        self.config = config
        self.manifest = config['manifest']
        self.routes = config['routes']

    @staticmethod
    def tube(position, route):
        centres = route['centres']
        start, end = subtract(centres[1], centres[0]), subtract(centres[-1], centres[-2])
        if dot(subtract(position, centres[0]), start) < -1e-12 or \
                dot(subtract(position, centres[-1]), end) > 1e-12:
            return None
        best, progress, length, total = math.inf, 0, 0, 0
        lengths = [math.dist(a, b) for a, b in zip(centres, centres[1:])]
        for a, b, segment_length in zip(centres, centres[1:], lengths):
            if segment_length <= 0:
                raise ValueError('Degenerate route segment')
            axis = subtract(b, a)
            fraction = min(1.0, max(0.0, dot(subtract(position, a), axis)/dot(axis, axis)))
            closest = tuple(x+fraction*d for x, d in zip(a, axis))
            distance = math.dist(position, closest)
            if distance < best:
                best, progress = distance, length+fraction*segment_length
            length += segment_length
            total += segment_length
        return {'centreline_distance_m': best, 'fraction': progress/total}

    def bin_contains(self, p):
        x, y = self.manifest['bin_center_m']
        low, high = self.manifest['hopper_upper_inner_bounds_m']
        top = self.manifest['initial_ball_bounds_world_m'][1][2] - self.manifest['robot_deck_world_z']
        if .208 <= p[2] <= top:
            return low[0] <= p[0] <= high[0] and low[1] <= p[1] <= high[1]
        distance = math.hypot(p[0]-x, p[1]-y)
        if -.022 <= p[2] <= .066:
            return distance <= .022
        if .065 <= p[2] < .208:
            if distance == 0:
                return True
            dx, dy = abs((p[0]-x)/distance), abs((p[1]-y)/distance)
            reach = min((high[0]-x)/dx if dx else math.inf,
                        (high[1]-y)/dy if dy else math.inf)
            radius = .022+(reach-.022)*(p[2]-.065)/(.208-.065)
            return distance <= radius
        return False

    def classify(self, ball, state):
        p = inverse_quaternion(subtract(ball['world_position'], state['model_position']),
                               state['model_orientation'])
        yaw, pitch = self.manifest['yaw_origin_m'], self.manifest['pitch_origin_m']
        signs = self.config['joint_axis_signs']
        yaw_neutral = about(p, yaw, rotate_z,
                            -signs['yaw_joint']*state['joints']['yaw_joint']['position'])
        head_neutral = about(yaw_neutral, pitch, rotate_y,
                             -signs['pitch_joint']*state['joints']['pitch_joint']['position'])
        coordinates = {'base': p, 'yaw': yaw_neutral, 'head': head_neutral}
        memberships, occupancy = set(), {}
        for name, route in self.routes.items():
            match = self.tube(coordinates[route['frame']], route)
            if match is not None and match['centreline_distance_m'] <= route['inner_radius_m']:
                memberships.add(name)
                match['whole_ball_centre_clearance'] = match['centreline_distance_m'] <= \
                    route['inner_radius_m']-self.config['ball_radius']
                occupancy[name] = match
        feeder = subtract(p, self.manifest['feeder_origin_m'])
        if -.006 <= feeder[2] <= .048 and \
                (math.hypot(feeder[0], feeder[1]) <= .091 or
                 0 <= feeder[0] <= .100 and -.094 <= feeder[1] <= -.030):
            memberships.add('feeder')
        if self.bin_contains(p):
            memberships.add('bin')
        local = ball['muzzle_local_position']
        if local is not None and abs(local[0]-self.config['muzzle_plane_x']) <= \
                self.config['ball_radius'] and math.hypot(local[1], local[2]) <= \
                self.config['aperture_radius']:
            memberships.add('muzzle')
        if not memberships:
            memberships.add('outside')
        primary = next(name for name in reversed(STAGES) if name in memberships)
        upper = any(n in memberships for n in ('yaw_elbow', 'head_elbow', 'head', 'muzzle')) or \
            ('riser' in occupancy and occupancy['riser']['fraction'] >= .5)
        above_rim = p[2]+self.config['ball_radius'] > \
            self.manifest['hopper_upper_inner_bounds_m'][1][2]
        return {'primary': primary, 'zones': sorted(memberships), 'tube_occupancy': occupancy,
                'upper_pipe': upper, 'above_hopper_rim': above_rim,
                'model_position': p}


class TrialAudit:
    def __init__(self, config, args):
        self.inventory = InventoryAudit(config['expected_names'])
        self.expected_count = len(config['expected_names'])
        self.geometry = Geometry(config)
        self.args = args
        self.initial = self.final = self.feed_baseline = None
        self.feeding_start = self.first_motion = None
        self.states = deque(maxlen=2048)
        self.joint_entities = None
        self.joint_initial = self.joint_final = None
        self.joint_min = {j: math.inf for j in JOINTS}
        self.joint_max = {j: -math.inf for j in JOINTS}
        self.max_wheel_speed = self.max_wheel_travel = 0
        self.events = []
        self.first_visits = {name: {} for name in config['expected_names']}
        self.clear_first_visits = {name: {} for name in config['expected_names']}
        self.priming_budget = priming_budget(config, args.feeder_speed, args.duration-args.settle)
        self.upper_entries = {}
        self.zone_max = Counter()
        self.unclassified_after_settle = set()
        self.previous_positions = None
        self.previous_time = None
        self.motion_history = deque()
        self.settled_at_command = False
        self.last_progress = None

    def consume_joint(self, state):
        entities = {n: j['entity'] for n, j in state['joints'].items()}
        if self.joint_entities is not None and entities != self.joint_entities:
            raise ValueError('A native mechanism joint entity was replaced')
        if self.states and state['sim_time'] < self.states[-1]['sim_time']:
            raise ValueError('Native joint-state time decreased')
        self.joint_entities = entities
        self.states.append(state)
        self.joint_initial = self.joint_initial or state
        self.joint_final = state
        for name, joint in state['joints'].items():
            self.joint_min[name] = min(self.joint_min[name], joint['position'])
            self.joint_max[name] = max(self.joint_max[name], joint['position'])
        if self.feeding_start is not None and state['sim_time'] >= self.feeding_start:
            wheels = [state['joints'][f'wheel_{i}_joint'] for i in range(1, 4)]
            self.max_wheel_speed = max(self.max_wheel_speed, *(abs(w['velocity']) for w in wheels))
            self.max_wheel_travel = max(self.max_wheel_travel,
                                       *(self.joint_max[f'wheel_{i}_joint'] -
                                         self.joint_min[f'wheel_{i}_joint'] for i in range(1, 4)))
            if state['joints']['feeder_joint']['velocity'] > .01 and self.first_motion is None:
                self.first_motion = state['sim_time']

    def start_feeding(self, now):
        self.feeding_start = now
        self.feed_baseline = self.final
        speeds = [speed for t, speed in self.motion_history if now-t <= .6]
        self.settled_at_command = bool(speeds) and \
            self.motion_history[0][0] <= now-.5 and max(speeds) <= self.args.settle_speed
        self.last_progress = now

    def consume_inventory(self, payload):
        if not self.inventory.consume(payload):
            return None
        if not payload['head_available']:
            raise ValueError('Monitored physical head link unavailable')
        if payload['discontinuous_samples']:
            raise ValueError('Monitor reported discontinuous physical ball samples')
        if not self.states:
            return None
        state = min(self.states, key=lambda s: abs(s['sim_time']-payload['sim_time']))
        if abs(state['sim_time']-payload['sim_time']) > self.args.joint_max_age:
            raise ValueError('Native joint feedback is too old for geometric classification')
        positions = {b['identity']: b['world_position'] for b in payload['balls']}
        now = payload['sim_time']
        if self.previous_positions is not None and now > self.previous_time:
            speed = max(math.dist(positions[n], self.previous_positions[n])/
                        (now-self.previous_time) for n in positions)
            self.motion_history.append((now, speed))
            while self.motion_history and now-self.motion_history[0][0] > 1.0:
                self.motion_history.popleft()
        self.previous_positions, self.previous_time = positions, now
        classifications = {b['identity']: self.geometry.classify(b, state) for b in payload['balls']}
        counts = Counter(c['primary'] for c in classifications.values())
        tubes = {name: sum(name in c['tube_occupancy'] for c in classifications.values()) for name in ROUTE}
        clearances = {name: sum(c['tube_occupancy'].get(name, {}).get('whole_ball_centre_clearance', False)
                               for c in classifications.values()) for name in ROUTE}
        phase = 'feeding' if self.feeding_start is not None else 'settling'
        for name, classification in classifications.items():
            for zone in classification['zones']:
                # A centre merely inside the nominal tube radius can overlap
                # its wall by almost a ball radius. Such a sample is diagnostic
                # occupancy, never sufficient evidence of clear transit.
                clear = zone not in ROUTE or classification['tube_occupancy'][zone][
                    'whole_ball_centre_clearance']
                if clear and zone not in self.clear_first_visits[name]:
                    self.clear_first_visits[name][zone] = now
                if zone not in self.first_visits[name]:
                    self.first_visits[name][zone] = now
                    self.events.append({'identity': name, 'zone': zone, 'sim_time': now,
                                        'phase': phase, 'world_position': positions[name]})
                    if phase == 'feeding' and zone in ROUTE:
                        self.last_progress = now
            if classification['upper_pipe'] and name not in self.upper_entries:
                self.upper_entries[name] = {'sim_time': now, 'phase': phase,
                                            'world_position': positions[name]}
            if phase == 'feeding' and classification['primary'] == 'outside':
                self.unclassified_after_settle.add(name)
        for zone, count in counts.items():
            self.zone_max[zone] = max(self.zone_max[zone], count)
        result = {'sim_time': now, 'phase': phase, 'joint_state_sim_time': state['sim_time'],
                  'zone_counts': {z: counts[z] for z in STAGES}, 'tube_centre_counts': tubes,
                  'tube_whole_ball_centre_clearance_counts': clearances,
                  'above_rim_ball_envelope_ids': sorted(n for n, c in classifications.items()
                                                       if c['above_hopper_rim'] and c['primary'] == 'bin'),
                  'positions': positions, 'entity_ids': self.inventory.initial_entities,
                  'classifications': classifications, 'native_joints': state['joints'],
                  'shot_count': payload['shot_count']}
        self.initial = self.initial or result
        self.final = result
        return result

    def report(self, completed, error):
        arrivals = {zone: sorted(n for n, visits in self.first_visits.items()
                                if self.feeding_start is not None and visits.get(zone, -math.inf) >
                                self.feeding_start) for zone in ROUTE}
        ordered = []
        for name, visits in self.clear_first_visits.items():
            times = [visits.get(z) for z in REQUIRED_TRANSIT]
            if all(t is not None for t in times) and all(a <= b for a, b in zip(times, times[1:])) and \
                    self.feeding_start is not None and times[-1] > self.feeding_start:
                ordered.append(name)
        travelled = self.joint_max['feeder_joint']-self.joint_min['feeder_joint'] \
            if self.joint_initial else 0
        wheels_stopped = self.joint_initial is not None and \
            self.max_wheel_speed <= .03 and self.max_wheel_travel <= .03
        inventory_pass = bool(completed and not error and self.inventory.samples >= 2)
        progress_idle = None if self.final is None or self.last_progress is None else \
            self.final['sim_time']-self.last_progress
        priming_pass = inventory_pass and self.first_motion is not None and travelled >= .05 and \
            wheels_stopped and bool(ordered) and not self.unclassified_after_settle and \
            self.settled_at_command
        return {'test': 'physical_feed_priming_trial', 'schema_version': 1,
                'trial_completed': completed, 'passed': priming_pass, 'priming_pass': priming_pass,
                'inventory_pass': inventory_pass, 'feeding_and_launch_validated': False,
                'hopper_capacity_validated': False, 'error': error,
                'expected_count': self.expected_count,
                'staged_diagnostic': self.expected_count != 100,
                'full_100_ball_priming_pass': priming_pass and self.expected_count == 100,
                'requested_sim_duration': self.args.duration,
                'requested_settling_duration': self.args.settle,
                'priming_planning_estimate': self.priming_budget,
                'settling_speed_threshold_m_s': self.args.settle_speed,
                'settling_stability_observed_at_command': self.settled_at_command,
                'feeder_command_rad_s': self.args.feeder_speed,
                'feeder_command_sim_time': self.feeding_start,
                'first_observed_feeder_motion_sim_time': self.first_motion,
                'observed_feeder_angle_range_rad': travelled,
                'wheels_stopped_observed': wheels_stopped,
                'maximum_launch_wheel_speed_rad_s': self.max_wheel_speed,
                'maximum_launch_wheel_angle_range_rad': self.max_wheel_travel,
                'joint_entity_ids': self.joint_entities,
                'initial_inventory': self.initial, 'settled_inventory': self.feed_baseline,
                'final_inventory': self.final,
                'maximum_primary_zone_counts': dict(self.zone_max),
                'new_route_arrival_ids': arrivals,
                'upper_pipe_entries': self.upper_entries,
                'ordered_feeder_to_head_transit_ids': sorted(ordered),
                'transit_acceptance_scope': 'Ordered samples with whole-ball centre clearance '
                    'in every required tube zone; settled start and stopped launch wheels.',
                'route_entry_events': self.events,
                'unclassified_after_settling_ids': sorted(self.unclassified_after_settle),
                'seconds_without_new_downstream_zone_entry': progress_idle,
                'possible_jam_or_unprimed_column': bool(progress_idle is not None and
                    progress_idle >= self.args.jam_window and self.first_motion is not None),
                'jam_confirmed': False,
                'jam_observation_scope': 'No newly observed downstream-zone entries; '
                                         'an unfilled column or stopped head can produce the same observation.',
                'native_joint_feedback_sample_period_sim_seconds': .02,
                'stop_reason': 'requested_duration' if completed else 'error_or_wall_timeout',
                'scope': 'Observed persistent ball centres and actuator motion; no contact '
                         'calibration, hopper capacity, launch or hardware-performance verdict.'}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--world', type=Path, default=PACKAGE/'worlds/physical_100.sdf')
    parser.add_argument('--model-dir', type=Path, default=PACKAGE/'models/pingpong_r10')
    parser.add_argument('--count', type=int, default=100,
                        help='Physical inventory target; values below100 are staged diagnostics')
    parser.add_argument('--duration', type=float, default=65.0,
                        help='Total measured simulation seconds, including settling')
    parser.add_argument('--settle', type=float, default=3.0, help='Stopped-feeder settling seconds')
    parser.add_argument('--timeout', type=float, default=120.0, help='Total wall-clock budget, seconds')
    parser.add_argument('--feeder-speed', type=float, default=.3, help='Physical feeder joint rad/s')
    parser.add_argument('--settle-speed', type=float, default=.05, help='Settling diagnostic max ball speed')
    parser.add_argument('--joint-max-age', type=float, default=.25, help='Maximum joint/pose time mismatch')
    parser.add_argument('--jam-window', type=float, default=5.0, help='No-progress observation window')
    parser.add_argument('--report', type=Path, default=Path('physical_feed_trial_report.json'))
    parser.add_argument('--dry-run', action='store_true', help='Validate source configuration only')
    args = parser.parse_args(argv)
    if not 1 <= args.count <= 100 or not all(math.isfinite(v) and v > 0 for v in
               (args.duration, args.settle, args.timeout, args.feeder_speed, args.settle_speed,
                args.joint_max_age, args.jam_window)) or args.settle >= args.duration or args.timeout <= 10:
        parser.error('Count1..100 and positive finite parameters required; settle < duration and timeout > 10')
    args.world, args.model_dir = args.world.resolve(), args.model_dir.resolve()
    args.report.parent.mkdir(parents=True, exist_ok=True)
    try:
        config = load_config(args.world, args.model_dir, args.count)
    except Exception as error:
        parser.error(str(error))
    if args.dry_run:
        report = {'test': 'physical_feed_priming_trial', 'status': 'not_run', 'passed': False,
                  'source_configuration_valid': True, 'physical_ball_count': args.count,
                  'staged_diagnostic': args.count != 100, 'full_100_ball_priming_pass': False,
                  'controllers': config['controllers'], 'inventory_topic': config['inventory_topic'],
                  'joint_topic': config['joint_topic'], 'routes': config['routes'],
                  'input_hashes': config['input_hashes'], 'requested_sim_duration': args.duration,
                  'requested_settling_duration': args.settle, 'feeder_command_rad_s': args.feeder_speed,
                  'wall_budget_seconds': args.timeout,
                  'priming_planning_estimate': priming_budget(config, args.feeder_speed,
                                                             args.duration-args.settle),
                  'scope': 'Static preflight only; no simulator, controller or transit result.'}
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps({k: report[k] for k in ('status', 'source_configuration_valid', 'passed')}))
        return 0
    audit = TrialAudit(config, args)
    started = time.monotonic()
    final_deadline, deadline = started+args.timeout, started+args.timeout-8.0
    env = dict(os.environ, IGN_PARTITION='physical-feed-trial-'+uuid.uuid4().hex)
    env['IGN_GAZEBO_RESOURCE_PATH'] = os.pathsep.join(filter(None,
        (str(args.model_dir.parent), env.get('IGN_GAZEBO_RESOURCE_PATH'))))
    messages = queue.Queue()
    server = inventory_sub = joint_sub = None
    completed, error, world_running, feeder_started = False, None, False, False
    pending = deque()
    stop_command_sent = False
    log_path, observations_path = args.report.with_suffix('.gazebo.log'), args.report.with_suffix('.observations.jsonl')
    try:
        with log_path.open('w') as log, observations_path.open('w') as observations:
            inventory_sub = subprocess.Popen(['ign', 'topic', '-e', '-t', config['inventory_topic']],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                bufsize=1, start_new_session=True)
            joint_sub = subprocess.Popen(['ign', 'topic', '-e', '-t', config['joint_topic']],
                env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                bufsize=1, start_new_session=True)
            threading.Thread(target=read_inventory, args=(inventory_sub.stdout, messages), daemon=True).start()
            threading.Thread(target=read_joints, args=(joint_sub.stdout, messages, config['model_name']),
                             daemon=True).start()
            server = subprocess.Popen(['ign', 'gazebo', '-s', str(args.world)], env=env,
                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            last_progress = 0.0
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError('Gazebo exited; inspect '+str(log_path))
                try:
                    kind, payload = messages.get(timeout=.1)
                except queue.Empty:
                    continue
                if kind in ('error', 'closed'):
                    raise RuntimeError(str(payload))
                if kind == 'joint':
                    audit.consume_joint(payload)
                else:
                    observations.write(json.dumps({'type': 'monitor', 'data': payload}, allow_nan=False)+'\n')
                    if not world_running:
                        if not audit.inventory.consume(payload):
                            continue
                        with ThreadPoolExecutor(max_workers=6) as pool:
                            commands = [pool.submit(publish, topic, 0.0, env, deadline)
                                        for topic in config['controllers'].values()]
                            for command in commands:
                                command.result()
                        response = cli(['ign', 'service', '-s', f'/world/{config["world_name"]}/control',
                            '--reqtype', 'ignition.msgs.WorldControl', '--reptype', 'ignition.msgs.Boolean',
                            '--timeout', '5000', '--req', 'pause: false'], env, deadline)
                        if not re.search(r'data:\s*true', response):
                            raise RuntimeError('Native world did not accept the unpause request')
                        world_running = True
                    pending.append(payload)
                while pending and audit.states:
                    snapshot = pending[0]
                    if snapshot['sim_time']-audit.states[-1]['sim_time'] > args.joint_max_age:
                        break
                    pending.popleft()
                    classified = audit.consume_inventory(snapshot)
                    if classified is None:
                        continue
                    observations.write(json.dumps({'type': 'classified', 'data': classified}, allow_nan=False)+'\n')
                    now = classified['sim_time']
                    if not feeder_started and now-audit.inventory.start_time >= args.settle:
                        audit.start_feeding(now)
                        publish(config['controllers']['feeder_joint'], args.feeder_speed, env, deadline)
                        feeder_started = True
                    if time.monotonic()-last_progress >= 10:
                        print(json.dumps({'sim_time': now, 'phase': classified['phase'],
                            'zones': classified['zone_counts'], 'above_rim': len(classified['above_rim_ball_envelope_ids'])}),
                            flush=True)
                        last_progress = time.monotonic()
                    if now-audit.inventory.start_time >= args.duration:
                        completed = True
                        break
                if completed:
                    break
            if not completed:
                raise TimeoutError(f'Wall budget exhausted at {audit.inventory.elapsed:.3f} measured sim seconds')
    except (Exception, KeyboardInterrupt) as exception:
        error = str(exception) or 'Interrupted'
    finally:
        if server is not None and server.poll() is None and world_running:
            try:
                publish(config['controllers']['feeder_joint'], 0.0, env, min(final_deadline, time.monotonic()+3))
                stop_command_sent = True
            except Exception:
                pass
        stop_process(server)
        stop_process(inventory_sub)
        stop_process(joint_sub)
    report = audit.report(completed, error)
    report.update({'world': str(args.world), 'transport_partition': env['IGN_PARTITION'],
                   'input_hashes': config['input_hashes'], 'routes': config['routes'],
                   'joint_axis_signs': config['joint_axis_signs'],
                   'controller_topics': config['controllers'], 'wall_duration': time.monotonic()-started,
                   'feeder_zero_command_sent_at_end': stop_command_sent,
                   'own_simulator_shutdown': server is not None and server.poll() is not None,
                   'native_observations_file': str(observations_path), 'simulator_log': str(log_path)})
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: report[k] for k in ('passed', 'inventory_pass', 'priming_pass',
        'ordered_feeder_to_head_transit_ids', 'possible_jam_or_unprimed_column', 'error')}, indent=2), flush=True)
    print(str(args.report.resolve()), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
