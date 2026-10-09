#!/usr/bin/env python3
"""Run bounded headless physics ablations inside the sourced Gazebo environment.

Call build_performance_worlds.py first. Each case gets its own transport
partition through the inventory harness. No diagnostic changes canonical CAD
or model geometry, and no actuator commands or ball pose resets are issued.
"""
import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--cases', nargs='+', default=['full_1', 'full_10', 'floor_only_100',
                                                    'hopper_only_100', 'static_robot_100', 'full_100'])
    parser.add_argument('--duration', type=float, default=2.)
    parser.add_argument('--timeout', type=float, default=60.)
    args = parser.parse_args()
    manifest = json.loads((args.directory/'benchmark_worlds_manifest.json').read_text())
    cases = {case['name']: case for case in manifest['worlds']}
    unknown = set(args.cases) - cases.keys()
    if unknown:
        parser.error(f'Unknown benchmark cases: {sorted(unknown)}')
    if args.duration <= 0 or args.timeout <= 0:
        parser.error('Duration and timeout must be positive')
    harness = Path(__file__).with_name('physical_inventory_smoke.py')
    env = dict(os.environ)
    env['IGN_GAZEBO_RESOURCE_PATH'] = os.pathsep.join(filter(None, (
        str((args.directory / manifest['snapshot_resource_path']).resolve()),
        env.get('IGN_GAZEBO_RESOURCE_PATH'))))
    results = []
    for case_name in args.cases:
        case = cases[case_name]
        report_path = args.directory / f'{case_name}_report.json'
        command = [sys.executable, str(harness), '--world', str(args.directory/case['file']),
                   '--count', str(case['count']), '--duration', str(args.duration),
                   '--timeout', str(args.timeout), '--report', str(report_path)]
        if case['scope'] == 'full':
            command += ['--containment-bounds', '-.272', '-.1638', '.840', '-.112', '.0462', '1.258']
        print(f'Running {case_name}: {case["count"]} balls, {case["robot_collisions"]} robot collisions',
              flush=True)
        started = time.monotonic()
        run = subprocess.run(command, env=env)
        report = json.loads(report_path.read_text()) if report_path.is_file() else {}
        fields = ['inventory_pass', 'passed', 'expected_count', 'sim_duration', 'wall_duration',
                  'requested_sim_duration',
                  'startup_to_first_snapshot_wall_s', 'observed_realtime_factor',
                  'final_containment_pass', 'final_outside_containment_ids',
                  'sampled_maximum_pair_penetration_m', 'final_max_sample_displacement_speed_mps', 'error']
        summary = {key: report.get(key) for key in fields}
        summary.update({'case': case_name, 'scope': case['scope'], 'report': report_path.name,
                        'returncode': run.returncode, 'runner_wall_s': time.monotonic()-started})
        results.append(summary)
        all_results = []
        for saved_case in manifest['worlds']:
            saved_path = args.directory / f'{saved_case["name"]}_report.json'
            if not saved_path.is_file():
                continue
            saved_report = json.loads(saved_path.read_text())
            saved_summary = {key: saved_report.get(key) for key in fields}
            saved_summary.update({'case': saved_case['name'], 'scope': saved_case['scope'],
                                  'report': saved_path.name,
                                  'robot_collisions': saved_case['robot_collisions'],
                                  'dart_constraint_solver': saved_case['dart_constraint_solver'],
                                  'dart_collision_detector': saved_case['dart_collision_detector']})
            all_results.append(saved_summary)
        (args.directory/'benchmark_summary.json').write_text(json.dumps({
            'purpose': manifest['purpose'], 'results': all_results}, indent=2) + '\n')
    print(json.dumps(results, indent=2))
    # Failed containment is an expected possible diagnostic outcome. The
    # reports carry per-case failures; runner exit only indicates execution.
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
