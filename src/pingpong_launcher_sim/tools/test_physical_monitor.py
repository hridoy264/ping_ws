#!/usr/bin/env python3
"""Native Fortress tests of the read-only ball observer using passive physics.

Run inside the Linux Fortress environment after building the monitor. Disposable
worlds use gravity and unconstrained or pendulum bodies; no velocity, force,
entity-spawn, or teleport command is sent. WorldControl only starts simulation.
These tests establish monitor behavior, not the robot's feeding performance.
"""

import argparse
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


TOPIC = '/pingpong/physical_ball_inventory'
IDENTITY = 'inventory_ball_001'
CASES = ('forward_gravity', 'reverse_gravity', 'moving_mouth_forward',
         'moving_mouth_stationary_ball', 'repeated_pendulum', 'missing_inventory')
DURATIONS = dict(zip(CASES, (0.8, 0.8, 1.2, 0.8, 6.0, 0.2)))


def element(parent, tag, text=None, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    if text is not None:
        child.text = str(text)
    return child


def sphere_link(model, name, radius, mass, collision_offset='0 0 0 0 0 0'):
    link = element(model, 'link', name=name)
    inertia = element(link, 'inertial')
    element(inertia, 'mass', mass)
    tensor = element(inertia, 'inertia')
    moment = 0.4 * mass * radius * radius
    for tag, value in (('ixx', moment), ('iyy', moment), ('izz', moment),
                       ('ixy', 0), ('ixz', 0), ('iyz', 0)):
        element(tensor, tag, value)
    collision = element(link, 'collision', name='sphere_collision')
    element(collision, 'pose', collision_offset)
    element(element(element(collision, 'geometry'), 'sphere'), 'radius', radius)
    return link


def pendulum(model, child, pivot_from_link):
    joint = element(model, 'joint', name='passive_pendulum', type='revolute')
    element(joint, 'parent', 'world')
    element(joint, 'child', child)
    element(joint, 'pose', pivot_from_link)
    axis = element(joint, 'axis')
    element(axis, 'xyz', '0 1 0')
    limit = element(axis, 'limit')
    element(limit, 'lower', -1000)
    element(limit, 'upper', 1000)
    element(element(axis, 'dynamics'), 'damping', 0)


def write_world(case, path):
    sdf = ET.Element('sdf', {'version': '1.8'})
    world_name = 'monitor_' + case
    world = element(sdf, 'world', name=world_name)
    element(world, 'gravity', '0 0 -9.81')
    physics = element(world, 'physics', name='1ms', type='ignored')
    element(physics, 'max_step_size', 0.001)
    element(physics, 'real_time_factor', 1.0)
    for filename, name in (
            ('ignition-gazebo-physics-system', 'ignition::gazebo::systems::Physics'),
            ('ignition-gazebo-user-commands-system', 'ignition::gazebo::systems::UserCommands')):
        element(world, 'plugin', filename=filename, name=name)

    head_z, head_x, pitch, aperture = 0.1, 0.0, math.pi / 2, 0.025
    dynamic_head = case.startswith('moving_mouth')
    if case in ('reverse_gravity', 'moving_mouth_stationary_ball'):
        pitch = -math.pi / 2
    if dynamic_head:
        head_z, aperture = 0.3, 0.10
    if case == 'moving_mouth_forward':
        head_x = -0.04
    if case == 'repeated_pendulum':
        head_z, pitch, aperture = 1.0 - math.hypot(0.2, 0.4), 0.0, 0.08
    launcher = element(world, 'model', name='monitor_fixture')
    element(launcher, 'static', str(not dynamic_head).lower())
    element(launcher, 'pose', f'{head_x} 0 {head_z} 0 0 0')
    # Offset the head's tiny collision so it cannot touch the test ball. The
    # monitor's geometric frame is at the link origin, independently of this.
    sphere_link(launcher, 'head_link', 0.005, 0.02, '0.15 0 0 0 0 0')
    if case == 'moving_mouth_forward':
        pendulum(launcher, 'head_link', '0.04 0 0.30 0 0 0')
    monitor = element(launcher, 'plugin', filename='libpingpong_physical_ball_monitor.so',
                      name='pingpong::PhysicalBallMonitor')
    for name, value in (
            ('head_link', 'head_link'), ('muzzle_pose', f'0 0 0 0 {pitch} 0'),
            ('aperture_radius', aperture), ('expected_ball_count', 1),
            ('publish_rate', 200), ('min_forward_speed', 0.01),
            ('ball_name_prefix', 'inventory_ball_'), ('ball_link', 'ball_link')):
        element(monitor, name, value)
    if case != 'missing_inventory':
        ball = element(world, 'model', name=IDENTITY)
        stationary = case == 'moving_mouth_stationary_ball'
        element(ball, 'static', str(stationary).lower())
        ball_x, ball_z = (0.0, 0.1) if stationary else (0.0, 0.5)
        if case == 'repeated_pendulum':
            ball_x, ball_z = -0.2, 0.6
        element(ball, 'pose', f'{ball_x} 0 {ball_z} 0 0 0')
        sphere_link(ball, 'ball_link', 0.02, 0.0027)
        if case == 'repeated_pendulum':
            pendulum(ball, 'ball_link', '0.2 0 0.4 0 0 0')
    ET.indent(sdf, space='  ')
    ET.ElementTree(sdf).write(path, encoding='utf-8', xml_declaration=True)
    return world_name, (head_x, head_z, pitch)


def decode(line):
    match = re.fullmatch(r'\s*data:\s*("(?:\\.|[^"\\])*")\s*', line)
    if match:
        return json.loads(json.loads(match.group(1)))
    return None


def pipe_reader(pipe, messages):
    try:
        for line in iter(pipe.readline, ''):
            messages.put(line)
    finally:
        messages.put(None)


def stop(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


class CaseAudit:
    def __init__(self, case, initial_mouth):
        self.case = case
        self.initial_mouth = initial_mouth
        self.samples = 0
        self.initial_entities = None
        self.last = None
        self.previous = None
        self.forward = self.reverse = 0
        self.mouth_deviation = 0.0
        self.maximum_count = 0
        self.first_world = None
        self.world_motion = 0.0

    def consume(self, payload):
        if payload.get('schema') != 'physical_ball_monitor/v1':
            raise ValueError('Unexpected monitor schema')
        if not math.isfinite(payload['sim_time']) or payload['reset_epoch'] != 0:
            raise ValueError('Invalid time or unexpected accounting reset')
        if not payload['head_available']:
            raise ValueError('Test head frame unavailable')
        if payload['duplicate_identity_count'] or payload['identity_replacement_count']:
            raise ValueError('Duplicate or replaced physical identity')
        if payload['unavailable_pose_count'] or payload['discontinuous_samples']:
            raise ValueError('Invalid or discontinuous native pose samples')
        if self.last and payload['sim_time'] < self.last['sim_time']:
            raise ValueError('Simulation time decreased')
        balls = payload['balls']
        if self.case == 'missing_inventory':
            if balls or payload['present_count'] != 0 or \
                    payload['unaccounted_expected_count'] != 1 or \
                    payload['inventory_matches_expected'] is not False:
                raise ValueError('Absent physical inventory was not reported as incomplete')
            if payload['shot_count']:
                raise ValueError('Missing inventory produced a shot')
        else:
            if len(balls) != 1 or balls[0]['identity'] != IDENTITY or \
                    payload['present_count'] != 1 or \
                    payload['inventory_matches_expected'] is not True:
                raise ValueError('Expected one conserved native ball')
            ball = balls[0]
            entities = (ball['model_entity'], ball['link_entity'])
            if min(entities) <= 0 or (self.initial_entities and entities != self.initial_entities):
                raise ValueError('Physical model/link identity changed')
            self.initial_entities = entities
            for field in ('world_position', 'muzzle_local_position'):
                if ball[field] is None or len(ball[field]) != 3 or \
                        not all(math.isfinite(v) for v in ball[field]):
                    raise ValueError('Non-finite observed ball coordinates')
            if not ball['present'] or payload['shot_count'] != int(ball['counted']):
                raise ValueError('Native ball count and identity disagree')
            if self.maximum_count > payload['shot_count']:
                raise ValueError('Deduplicated count decreased')
            self.maximum_count = max(self.maximum_count, payload['shot_count'])
            local = ball['muzzle_local_position']
            position = ball['world_position']
            if self.first_world is None:
                self.first_world = position
            self.world_motion = max(self.world_motion, math.dist(position, self.first_world))
            head_x, head_z, pitch = self.initial_mouth
            dx, dz = position[0] - head_x, position[2] - head_z
            stationary_frame = (math.cos(pitch)*dx - math.sin(pitch)*dz,
                                position[1], math.sin(pitch)*dx + math.cos(pitch)*dz)
            self.mouth_deviation = max(self.mouth_deviation, math.dist(local, stationary_frame))
            if self.previous is not None:
                old = self.previous['muzzle_local_position']
                if old[0] <= 0 < local[0]:
                    self.forward += 1
                if old[0] > 0 >= local[0]:
                    self.reverse += 1
            if ball['counted']:
                crossing = ball['crossing_muzzle_local_position']
                if crossing is None or not all(math.isfinite(v) for v in crossing) or \
                        abs(crossing[0]) > 1e-8 or ball['crossing_sim_time'] is None:
                    raise ValueError('Count lacks a finite measured plane crossing')
            self.previous = ball
        self.last = payload
        self.samples += 1

    def verdict(self):
        if self.samples < 3 or self.last is None:
            raise ValueError('Insufficient native observations')
        if self.case in ('forward_gravity', 'moving_mouth_forward'):
            if self.maximum_count != 1 or self.forward < 1 or self.world_motion < 0.1:
                raise ValueError('Dynamic ball did not produce one measured forward crossing')
        elif self.case == 'reverse_gravity':
            if self.maximum_count or self.reverse < 1:
                raise ValueError('Reverse physical crossing was counted or not observed')
        elif self.case == 'moving_mouth_stationary_ball':
            if self.maximum_count or self.forward < 1 or self.world_motion > 1e-8:
                raise ValueError('Moving mouth/stationary ball rejection was not established')
        elif self.case == 'repeated_pendulum':
            if self.maximum_count != 1 or self.forward < 2 or self.reverse < 1:
                raise ValueError('Repeated physical crossings did not preserve identity dedup')
        if self.case.startswith('moving_mouth') and self.mouth_deviation < 0.001:
            raise ValueError('Head-local data did not show a moving mouth frame')
        return {
            'case': self.case, 'passed': True, 'samples': self.samples,
            'sim_end': self.last['sim_time'], 'persistent_entity_ids': self.initial_entities,
            'physical_shot_count': self.maximum_count,
            'observed_forward_plane_crossings': self.forward,
            'observed_reverse_plane_crossings': self.reverse,
            'maximum_ball_displacement_m': self.world_motion,
            'maximum_deviation_from_initial_mouth_frame_m': self.mouth_deviation,
            'missing_inventory_failure_detected': self.case == 'missing_inventory',
        }


def run_case(case, directory, timeout):
    world_path = directory / f'{case}.sdf'
    world_name, initial_mouth = write_world(case, world_path)
    audit = CaseAudit(case, initial_mouth)
    env = dict(os.environ, IGN_PARTITION='monitor-test-' + uuid.uuid4().hex)
    messages = queue.Queue()
    server = subscriber = None
    deadline = time.monotonic() + timeout
    started = False
    diagnostics = []
    with (directory / f'{case}.gazebo.log').open('w') as log, \
            (directory / f'{case}.observations.jsonl').open('w') as observations:
        try:
            # Listen before startup so the initial paused inventory cannot be
            # missed. Paused PostUpdate has a fixed simulation time.
            subscriber = subprocess.Popen(['ign', 'topic', '-t', TOPIC, '-e'],
                                          env=env, stdout=subprocess.PIPE,
                                          stderr=subprocess.STDOUT, text=True,
                                          bufsize=1, start_new_session=True)
            threading.Thread(target=pipe_reader, args=(subscriber.stdout, messages),
                             daemon=True).start()
            server = subprocess.Popen(['ign', 'gazebo', '-s', str(world_path)],
                                      env=env, stdout=log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            while time.monotonic() < deadline:
                if server.poll() is not None:
                    raise RuntimeError(f'Gazebo exited {server.returncode}; see case log')
                try:
                    line = messages.get(timeout=0.2)
                except queue.Empty:
                    continue
                if line is None:
                    raise RuntimeError('Native inventory subscriber exited')
                payload = decode(line)
                if payload is None:
                    diagnostics.append(line.rstrip())
                    continue
                observations.write(json.dumps(payload, allow_nan=False) + '\n')
                audit.consume(payload)
                if not started:
                    if payload['shot_count'] != 0:
                        raise ValueError('Paused baseline already contains a shot')
                    command = subprocess.run([
                        'ign', 'service', '-s', f'/world/{world_name}/control',
                        '--reqtype', 'ignition.msgs.WorldControl',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '5000',
                        '--req', 'pause: false'], env=env, text=True,
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=8)
                    if command.returncode or not re.search(r'data:\s*true', command.stdout):
                        raise RuntimeError('Could not unpause disposable physics world: ' + command.stdout)
                    started = True
                if payload['sim_time'] >= DURATIONS[case]:
                    return audit.verdict()
            raise RuntimeError('Native observation timeout; see simulator log')
        finally:
            stop(server)
            stop(subscriber)
            if diagnostics:
                (directory / f'{case}.subscriber.log').write_text('\n'.join(diagnostics) + '\n')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, action='append', help='Run only these cases')
    parser.add_argument('--timeout', type=float, default=30.0, help='Wall seconds per case')
    parser.add_argument('--report', type=Path, default=Path('physical_monitor_native_report.json'))
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error('--timeout must be finite and positive')
    args.report.parent.mkdir(parents=True, exist_ok=True)
    directory = args.report.parent / (args.report.stem + '_artifacts')
    directory.mkdir(parents=True, exist_ok=True)
    results = []
    for case in args.case or CASES:
        try:
            result = run_case(case, directory, args.timeout)
        except Exception as error:
            result = {'case': case, 'passed': False, 'error': str(error)}
        results.append(result)
        print(json.dumps(result, allow_nan=False), flush=True)
    report = {
        'test': 'physical_ball_monitor_native', 'passed': all(r['passed'] for r in results),
        'cases': results, 'artifact_directory': str(directory),
        'scope': 'Native passive-physics monitor tests only; no robot feeding or contact verdict.',
    }
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
