"""Reject misleading feed verdicts and respect actual signed joint axes."""
import copy
import math
from pathlib import Path
import sys
import unittest
from types import SimpleNamespace

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE/'tools'))
from physical_feed_trial import (Geometry, TrialAudit, JOINTS, REQUIRED_TRANSIT,
                                 about, load_config, priming_budget, rotate_y, rotate_z)


class FeedTrialTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config(PACKAGE/'worlds/physical_100.sdf',
                                  PACKAGE/'models/pingpong_r10')

    def test_old_short_run_and_small_inventory_do_not_cover_cold_prime(self):
        estimate = priming_budget(self.config, .3, 12)
        self.assertFalse(estimate['duration_covers_ideal_head_arrival'])
        self.assertGreater(estimate['ideal_head_inlet_time_s'], 35)
        small = dict(self.config, expected_names=self.config['expected_names'][:10])
        self.assertFalse(priming_budget(small, .3, 60)[
            'inventory_can_form_estimated_head_column'])

    def test_route_classification_uses_both_actual_axis_signs(self):
        for yaw_sign in (-1, 1):
            for pitch_sign in (-1, 1):
                config = copy.deepcopy(self.config)
                config['joint_axis_signs'].update(yaw_joint=yaw_sign,pitch_joint=pitch_sign)
                model_position = (.3,-.7,.91)
                model_angle = .4
                yaw_angle, pitch_angle = .3, -.2
                local = (.15,0,.185)
                p = about(local, config['manifest']['pitch_origin_m'], rotate_y,
                          pitch_sign*pitch_angle)
                p = about(p, config['manifest']['yaw_origin_m'], rotate_z,
                          yaw_sign*yaw_angle)
                p = rotate_z(p, model_angle)
                world = tuple(a+b for a,b in zip(p,model_position))
                state = {'model_position': model_position,
                         'model_orientation': (math.cos(model_angle/2),0,0,math.sin(model_angle/2)),
                         'joints': {'yaw_joint': {'position': yaw_angle},
                                    'pitch_joint': {'position': pitch_angle}}}
                result = Geometry(config).classify(
                    {'world_position': world, 'muzzle_local_position': None},state)
                self.assertIn('head',result['zones'])
                self.assertTrue(result['tube_occupancy']['head']['whole_ball_centre_clearance'])

    def test_centre_inside_tube_but_ball_inside_wall_is_not_clear(self):
        state={'model_position': (0,0,0), 'model_orientation': (1,0,0,0),
               'joints': {'yaw_joint': {'position': 0},'pitch_joint': {'position': 0}}}
        result=Geometry(self.config).classify(
            {'world_position': (.15,.01,.185), 'muzzle_local_position': None},state)
        self.assertIn('head',result['zones'])
        self.assertFalse(result['tube_occupancy']['head']['whole_ball_centre_clearance'])

    def test_false_transit_and_unsettled_start_cannot_pass(self):
        args=SimpleNamespace(feeder_speed=.3,duration=65,settle=3,settle_speed=.05,jam_window=5)
        audit=TrialAudit(self.config,args)
        audit.inventory.samples=2
        audit.joint_initial={'present': True}
        audit.joint_min={j:0 for j in JOINTS}
        audit.joint_max={j:0 for j in JOINTS}
        audit.joint_max['feeder_joint']=1
        audit.first_motion=3.1
        audit.feeding_start=3
        audit.settled_at_command=True
        identity=self.config['expected_names'][0]
        visits={zone:4+i for i,zone in enumerate(REQUIRED_TRANSIT)}
        audit.first_visits[identity]=visits
        self.assertFalse(audit.report(True,None)['priming_pass'])
        audit.clear_first_visits[identity]=visits
        self.assertTrue(audit.report(True,None)['priming_pass'])
        audit.settled_at_command=False
        self.assertFalse(audit.report(True,None)['priming_pass'])


if __name__ == '__main__':
    unittest.main()
