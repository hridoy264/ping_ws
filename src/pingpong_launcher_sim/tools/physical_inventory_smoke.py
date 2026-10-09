#!/usr/bin/env python3
"""Observe conserved physical balls in a Fortress world; never request a spawn.

Uses `ign topic -e` and the physical monitor's StringMsg JSON. A successful run
with --min-shots 0 checks inventory only and always reports feeding_pass=false.
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
import sys
import threading
import time


TOPIC = '/pingpong/physical_ball_inventory'


def decode_topic_line(line):
    """Decode protobuf text StringMsg output, ignoring CLI diagnostics."""
    match = re.fullmatch(r'\s*data:\s*("(?:\\.|[^"\\])*")\s*', line)
    if not match:
        if re.match(r'\s*data:', line):
            raise ValueError('Malformed StringMsg data line')
        return None
    outer = json.loads(match.group(1))
    payload = json.loads(outer)
    if not isinstance(payload, dict):
        raise ValueError('Physical monitor payload must be a JSON object')
    return payload


def finite_number(value, field):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'{field} must be finite')
    return float(value)


class InventoryAudit:
    """Audit actual monitor snapshots; neither creates entities nor moves them."""

    def __init__(self, expected_names, min_shots=0, allowed_bounds=None,
                 containment_bounds=None, ball_radius=.020, max_penetration=.001):
        self.expected = set(expected_names)
        self.min_shots = min_shots
        self.allowed_bounds = allowed_bounds
        self.containment_bounds = containment_bounds
        self.ball_radius = ball_radius
        self.max_penetration = max_penetration
        self.minimum_pair_spacing = math.inf
        self.minimum_pair_names = None
        self.last_sample_speeds = {}
        self.samples = 0
        self.start_time = None
        self.last_time = None
        self.initial_crossed = None
        self.last_crossed = set()
        self.initial_shots = None
        self.last_shots = None
        self.bounds_min = [math.inf] * 3
        self.bounds_max = [-math.inf] * 3
        self.first_positions = {}
        self.last_positions = {}
        self.initial_entities = None
        self.reset_epoch = None
        self.startup_snapshots = 0
        self.latest = None

    def consume(self, payload):
        # The monitor protocol is documented beside its implementation. Keeping
        # these names explicit prevents accidentally consuming abstract counts.
        self.latest = payload
        if payload['schema'] != 'physical_ball_monitor/v1':
            raise ValueError('Expected physical_ball_monitor/v1; refusing another count source')
        now = finite_number(payload['sim_time'], 'sim_time')
        if now < 0:
            raise ValueError('Simulation time cannot be negative')
        balls = payload['balls']
        if not isinstance(balls, list):
            raise ValueError('balls must be an array')
        positions = {}
        crossed = set()
        identities = set()
        entities = {}
        for ball in balls:
            name = ball['identity']
            if not isinstance(name, str) or not name:
                raise ValueError('Ball name must be a nonempty string')
            if name in identities:
                raise ValueError(f'Duplicate live ball identity: {name}')
            identities.add(name)
            marked = ball['counted']
            if not isinstance(marked, bool) or not isinstance(ball['present'], bool):
                raise ValueError(f'{name}: counted and present must be boolean')
            if marked:
                crossed.add(name)
            for field in ('model_entity', 'link_entity'):
                if isinstance(ball[field], bool) or not isinstance(ball[field], int) or ball[field] <= 0:
                    raise ValueError(f'{name}.{field} must be a positive entity ID')
            entities[name] = (ball['model_entity'], ball['link_entity'])
            if not ball['present'] or ball['world_position'] is None:
                continue
            position = ball['world_position']
            if not isinstance(position, list) or len(position) != 3:
                raise ValueError(f'{name}: expected three world-position coordinates')
            positions[name] = tuple(finite_number(v, f'{name}.position') for v in position)
        count = payload['shot_count']
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError('Physical monitor shot_count must be a nonnegative integer')
        if count != len(crossed):
            raise ValueError('Physical shot count disagrees with individually crossed identities')
        present = set(positions)
        missing, unexpected = self.expected - present, identities - self.expected
        if unexpected:
            raise ValueError(f'Unexpected inventory identities: {sorted(unexpected)}')
        for field in ('duplicate_identity_count', 'identity_replacement_count'):
            if payload[field] != 0:
                raise ValueError(f'Physical monitor reported {field}={payload[field]}')
        if len({value[0] for value in entities.values()}) != len(entities) or \
                len({value[1] for value in entities.values()}) != len(entities):
            raise ValueError('Distinct ball names resolve to the same physical entity')
        epoch = payload['reset_epoch']
        if isinstance(epoch, bool) or not isinstance(epoch, int) or epoch < 0:
            raise ValueError('reset_epoch must be a nonnegative integer')
        if self.reset_epoch is not None and epoch != self.reset_epoch:
            raise ValueError('Physical monitor reset during measured run')
        if missing:
            if self.start_time is None:
                self.startup_snapshots += 1
                return False
            raise ValueError(f'Persistent balls disappeared: {sorted(missing)}')
        if payload['expected_ball_count'] != len(self.expected) or \
                payload['present_count'] != len(self.expected) or \
                payload['inventory_matches_expected'] is not True:
            raise ValueError('Physical monitor inventory totals disagree with expected identities')
        if self.min_shots and payload['head_available'] is not True:
            raise ValueError('Feeding test requires an available monitored head link')
        if self.initial_entities is not None and entities != self.initial_entities:
            raise ValueError('A persistent ball model/link entity was replaced')
        if self.last_time is not None and now < self.last_time:
            raise ValueError('Simulation/monitor reset during measured run')
        if not self.last_crossed.issubset(crossed):
            raise ValueError('Previously crossed identities disappeared from monitor history')
        if self.last_shots is not None and count < self.last_shots:
            raise ValueError('Physical shot counter decreased during measured run')
        if self.allowed_bounds:
            for name, position in positions.items():
                if not all(lo <= value <= hi for value, lo, hi in
                           zip(position, self.allowed_bounds[:3], self.allowed_bounds[3:])):
                    raise ValueError(f'{name} escaped allowed world centre bounds: {position}')
        if self.start_time is None:
            self.start_time = now
            self.initial_shots = count
            self.initial_crossed = set(crossed)
            self.first_positions = positions.copy()
            self.initial_entities = entities.copy()
            self.reset_epoch = epoch
        if self.last_time is not None and now > self.last_time:
            self.last_sample_speeds = {name: math.dist(position, self.last_positions[name]) /
                                      (now - self.last_time) for name, position in positions.items()}
        pairs = list(positions.items())
        for i, (name, position) in enumerate(pairs):
            for other, other_position in pairs[i+1:]:
                distance = math.dist(position, other_position)
                if distance < self.minimum_pair_spacing:
                    self.minimum_pair_spacing = distance
                    self.minimum_pair_names = (name, other)
        self.last_time = now
        self.last_shots = count
        self.last_crossed = crossed
        self.last_positions = positions
        self.samples += 1
        for position in positions.values():
            for axis, value in enumerate(position):
                self.bounds_min[axis] = min(self.bounds_min[axis], value)
                self.bounds_max[axis] = max(self.bounds_max[axis], value)
        return True

    @property
    def elapsed(self):
        return 0.0 if self.start_time is None else self.last_time - self.start_time

    def report(self, completed=False, error=None):
        new_crossings = sorted(self.last_crossed - (self.initial_crossed or set()))
        inventory_pass = completed and not error and self.samples >= 2
        target_met = len(new_crossings) >= self.min_shots
        penetration = max(0., 2*self.ball_radius-self.minimum_pair_spacing) if self.samples else None
        pair_clearance_pass = inventory_pass and penetration <= self.max_penetration
        outside=[]
        if self.containment_bounds:
            for name, position in self.last_positions.items():
                if not all(lo+self.ball_radius-.0001 <= value <= hi-self.ball_radius+.0001
                           for value,lo,hi in zip(position,self.containment_bounds[:3],self.containment_bounds[3:])):
                    outside.append(name)
        containment_pass = inventory_pass and not outside if self.containment_bounds else None
        checks_pass = pair_clearance_pass and containment_pass is not False
        feeding_pass = checks_pass and self.min_shots > 0 and target_met
        return {
            'test': 'physical_inventory_smoke', 'schema_version': 1,
            'inventory_pass': inventory_pass, 'feeding_pass': feeding_pass,
            'passed': checks_pass and target_met,
            'feeding_test_requested': self.min_shots > 0,
            'expected_count': len(self.expected), 'expected_ids': sorted(self.expected),
            'measured_snapshots': self.samples, 'incomplete_startup_snapshots': self.startup_snapshots,
            'sim_start': self.start_time, 'sim_end': self.last_time,
            'sim_duration': self.elapsed, 'minimum_requested_new_shots': self.min_shots,
            'initial_physical_shot_count': self.initial_shots,
            'final_physical_shot_count': self.last_shots,
            'new_crossing_count': len(new_crossings), 'new_crossing_ids': new_crossings,
            'world_position_min': self.bounds_min if self.samples else None,
            'world_position_max': self.bounds_max if self.samples else None,
            'initial_positions': self.first_positions, 'final_positions': self.last_positions,
            'reset_epoch': self.reset_epoch, 'persistent_entity_ids': self.initial_entities,
            'sampled_pair_clearance_pass': pair_clearance_pass,
            'sampled_minimum_pair_spacing_m': self.minimum_pair_spacing if math.isfinite(self.minimum_pair_spacing) else None,
            'sampled_closest_pair': self.minimum_pair_names,
            'sampled_maximum_pair_penetration_m': penetration,
            'maximum_permitted_pair_penetration_m': self.max_penetration,
            'final_containment_pass': containment_pass,
            'final_outside_containment_ids': sorted(outside),
            'containment_world_bounds': self.containment_bounds,
            'final_max_sample_displacement_speed_mps': max(self.last_sample_speeds.values(),default=None),
            'error': error,
            'scope': 'Observed inventory identities and muzzle crossings only; '
                     'no contact calibration or hardware performance claim.',
        }


def pipe_reader(pipe, messages):
    try:
        for line in iter(pipe.readline, ''):
            messages.put(line)
    finally:
        messages.put(None)


def terminate_process(process):
    if process is None or process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGKILL)
        process.wait(timeout=3)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--world', type=Path, help='Start this world headlessly and unpaused')
    mode.add_argument('--attach', action='store_true', help='Observe an existing running world')
    parser.add_argument('--topic', default=TOPIC)
    parser.add_argument('--count', type=int, default=100)
    parser.add_argument('--prefix', default='inventory_ball_')
    parser.add_argument('--duration', type=float, default=30.0, help='Measured simulation seconds')
    parser.add_argument('--timeout', type=float, default=180.0, help='Overall wall-clock timeout')
    parser.add_argument('--min-shots', type=int, default=0,
                        help='New observed muzzle crossings required; 0 tests inventory only')
    parser.add_argument('--allowed-bounds', nargs=6, type=float,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'),
                        help='Optional permitted ball-centre world bounds in metres')
    parser.add_argument('--containment-bounds', nargs=6, type=float,
                        metavar=('XMIN', 'YMIN', 'ZMIN', 'XMAX', 'YMAX', 'ZMAX'),
                        help='Optional final sphere-envelope bounds, e.g. hopper rim after settling')
    parser.add_argument('--ball-radius', type=float, default=.020)
    parser.add_argument('--max-penetration', type=float, default=.001,
                        help='Maximum sampled ball-to-ball penetration allowed, metres')
    parser.add_argument('--partition', help='Ignition transport partition (must match for --attach)')
    parser.add_argument('--report', type=Path, default=Path('physical_inventory_report.json'))
    args = parser.parse_args(argv)
    if args.count < 1 or not 0 <= args.min_shots <= args.count:
        parser.error('--count must be positive; --min-shots must be within 0..count')
    if not all(math.isfinite(v) and v > 0 for v in (args.duration, args.timeout)):
        parser.error('--duration and --timeout must be finite and positive')
    if args.allowed_bounds and (not all(math.isfinite(v) for v in args.allowed_bounds) or
                                   not all(lo < hi for lo, hi in zip(args.allowed_bounds[:3],
                                                                   args.allowed_bounds[3:]))):
        parser.error('--allowed-bounds requires finite minima below maxima')
    if args.containment_bounds and (not all(math.isfinite(v) for v in args.containment_bounds) or
                                   not all(lo < hi for lo,hi in zip(args.containment_bounds[:3],
                                                                  args.containment_bounds[3:]))):
        parser.error('--containment-bounds requires finite minima below maxima')
    if not math.isfinite(args.ball_radius) or args.ball_radius <= 0 or \
            not math.isfinite(args.max_penetration) or args.max_penetration < 0:
        parser.error('Ball radius must be positive and penetration tolerance nonnegative; both finite')
    if args.world and not args.world.is_file():
        parser.error('--world file does not exist')
    names = tuple(f'{args.prefix}{i:03d}' for i in range(1, args.count + 1))
    audit = InventoryAudit(names, args.min_shots, args.allowed_bounds,
                           args.containment_bounds,args.ball_radius,args.max_penetration)
    env = dict(os.environ)
    if args.partition:
        env['IGN_PARTITION'] = args.partition
    elif args.world:
        env['IGN_PARTITION'] = f'physical-inventory-{os.getpid()}-{int(time.time())}'
    server = subscriber = log = None
    error = None
    completed = False
    started = time.monotonic()
    diagnostic_lines = []
    performance_samples = []
    log_path = args.report.with_suffix('.gazebo.log')
    try:
        if args.world:
            log = log_path.open('w')
            server = subprocess.Popen(['ign', 'gazebo', '-s', '-r', str(args.world.resolve())],
                                      stdout=log, stderr=subprocess.STDOUT, env=env,
                                      start_new_session=True)
            print(f'Started Gazebo; log: {log_path}', flush=True)
        subscriber = subprocess.Popen(['ign', 'topic', '-e', '-t', args.topic],
                                      stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                      text=True, bufsize=1, env=env, start_new_session=True)
        messages = queue.Queue()
        threading.Thread(target=pipe_reader, args=(subscriber.stdout, messages), daemon=True).start()
        announced = False
        last_progress = 0.0
        while time.monotonic() - started < args.timeout:
            if server is not None and server.poll() is not None:
                raise RuntimeError(f'Gazebo exited with status {server.returncode}; inspect {log_path}')
            try:
                line = messages.get(timeout=0.1)
            except queue.Empty:
                continue
            if line is None:
                raise RuntimeError(f'ign topic ended; diagnostics: {diagnostic_lines[-5:]}')
            payload = decode_topic_line(line)
            if payload is None:
                if line.strip():
                    diagnostic_lines.append(line.rstrip())
                    diagnostic_lines = diagnostic_lines[-20:]
                continue
            if not audit.consume(payload):
                continue
            performance_samples.append({'sim_time': audit.last_time,
                                        'wall_since_start_s': time.monotonic() - started})
            if not announced:
                print(f'Observed all {args.count} IDs at sim time {audit.start_time:.3f}', flush=True)
                announced = True
            if time.monotonic() - last_progress >= 10:
                print(f'{audit.elapsed:.1f}/{args.duration:.1f} simulated seconds; '
                      f'{len(audit.last_crossed - audit.initial_crossed)} new physical crossings', flush=True)
                last_progress = time.monotonic()
            if audit.elapsed >= args.duration:
                completed = True
                break
        if not completed:
            raise TimeoutError(f'Wall timeout after {args.timeout:g}s; observed '
                               f'{audit.samples} complete snapshots and {audit.elapsed:.3f} simulated seconds')
    except (OSError, RuntimeError, ValueError, KeyError, TypeError, TimeoutError) as exc:
        error = str(exc)
    except KeyboardInterrupt:
        error = 'Interrupted by user'
    finally:
        terminate_process(subscriber)
        terminate_process(server)
        if log:
            log.close()
    report = audit.report(completed, error)
    report.update({'wall_duration': time.monotonic() - started,
                   'topic': args.topic, 'transport_partition': env.get('IGN_PARTITION'),
                   'world': str(args.world.resolve()) if args.world else None,
                   'attached': args.attach, 'diagnostics': diagnostic_lines,
                   'requested_sim_duration': args.duration})
    report['performance_samples'] = performance_samples
    report['startup_to_first_snapshot_wall_s'] = (performance_samples[0]['wall_since_start_s']
                                                 if performance_samples else None)
    sampled_wall = (performance_samples[-1]['wall_since_start_s'] -
                    performance_samples[0]['wall_since_start_s']) if len(performance_samples) > 1 else 0.
    report['observed_realtime_factor'] = audit.elapsed / sampled_wall if sampled_wall > 0 else None
    if error and audit.latest is not None:
        # Preserve the failing monitor snapshot for diagnosis. JSON forbids NaN
        # in reports; stringify any deliberately rejected nonfinite numbers.
        report['last_monitor_snapshot'] = json.loads(json.dumps(audit.latest),
                                                     parse_constant=lambda value: value)
    args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: report[key] for key in
                      ('passed', 'inventory_pass', 'feeding_pass', 'new_crossing_count', 'error')}, indent=2))
    print(f'Report: {args.report.resolve()}', flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
