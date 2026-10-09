"""Finite inventory contracts; these do not substitute for Gazebo contact tests."""
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pingpong_launcher_sim.inventory import (  # noqa: E402
    AABB, BallLedger, BallSpec, Phase, append_to_world, inventory_sdf, pack_balls,
)


class PackingTests(unittest.TestCase):
    def test_hundred_jittered_spheres_fit_and_do_not_overlap(self):
        bounds = AABB((-0.31, -0.14, 0.2), (-0.01, 0.16, 0.5))
        for seed in (0, 7, 913):
            packing = pack_balls(bounds, jitter=0.002, seed=seed)
            self.assertEqual(len(packing.balls), 100)
            self.assertEqual(len({ball.name for ball in packing.balls}), 100)
            margin = packing.spec.radius + packing.wall_clearance
            for ball in packing.balls:
                for value, lo, hi in zip(ball.position, bounds.minimum, bounds.maximum):
                    self.assertGreaterEqual(value, lo + margin - 1e-12)
                    self.assertLessEqual(value, hi - margin + 1e-12)
            for i, ball in enumerate(packing.balls):
                for other in packing.balls[i + 1:]:
                    self.assertGreaterEqual(math.dist(ball.position, other.position),
                                            2 * packing.spec.radius + packing.surface_gap - 1e-12)

    def test_seed_is_reproducible_and_changes_jitter(self):
        bounds = AABB((0, 0, 0), (0.3, 0.3, 0.3))
        a = pack_balls(bounds, jitter=0.001, seed=17)
        self.assertEqual(a, pack_balls(bounds, jitter=0.001, seed=17))
        self.assertNotEqual(a.balls, pack_balls(bounds, jitter=0.001, seed=18).balls)
        self.assertEqual(pack_balls(bounds, seed=17).balls, pack_balls(bounds, seed=18).balls)

    def test_exact_capacity_and_insufficient_space(self):
        # Five by five by four positions, 41 mm pitch and 1 mm wall clearance.
        bounds = AABB((0, 0, 0), (0.206, 0.206, 0.165))
        packing = pack_balls(bounds)
        self.assertEqual(packing.grid_shape, (5, 5, 4))
        self.assertEqual(packing.capacity, 100)
        with self.assertRaisesRegex(ValueError, 'fewer than requested 101'):
            pack_balls(bounds, 101)
        with self.assertRaises(ValueError):
            pack_balls(AABB((0, 0, 0), (0.04, 0.04, 0.04)), 1)

    def test_invalid_inputs_fail_before_placement(self):
        for bad in (float('nan'), float('inf'), -1, 0, True):
            with self.assertRaises(ValueError):
                BallSpec(radius=bad)
        for bounds in (((0, 0, 0), (1, 0, 1)), ((0, 0, 0), (1, 1, float('nan')))):
            with self.assertRaises(ValueError):
                AABB(*bounds)
        bounds = AABB((0, 0, 0), (1, 1, 1))
        for count in (0, -1, 1.5, True):
            with self.assertRaises(ValueError):
                pack_balls(bounds, count)
        for kwargs in ({'surface_gap': -0.1}, {'jitter': float('nan')},
                       {'seed': 1.5}, {'prefix': 'bad name'}):
            with self.assertRaises(ValueError):
                pack_balls(bounds, **kwargs)


class SDFTests(unittest.TestCase):
    def setUp(self):
        self.packing = pack_balls(AABB((0, 0, 1), (0.3, 0.3, 1.3)))

    def test_models_are_physical_hollow_spheres_without_expiration(self):
        root = inventory_sdf(self.packing)
        self.assertEqual(len(root.findall('model')), 100)
        self.assertEqual(len(root.findall('.//plugin')), 0)
        for model in root.findall('model'):
            self.assertEqual(model.findtext('static'), 'false')
            self.assertEqual(model.findtext('link/gravity'), 'true')
            self.assertAlmostEqual(float(model.findtext('link/inertial/mass')), 0.0027)
            for axis in ('ixx', 'iyy', 'izz'):
                self.assertAlmostEqual(float(model.findtext('link/inertial/inertia/' + axis)), 7.2e-7)
            self.assertAlmostEqual(float(model.findtext('link/collision/geometry/sphere/radius')), .020)
            self.assertEqual(len(model.findall('.//collision')), 1)

    def test_world_copy_preserves_existing_content_and_identity(self):
        original = ET.fromstring('<sdf version="1.8"><world name="custom_world">'
                                 '<gravity>0 0 -9.81</gravity><include><uri>model://robot</uri>'
                                 '</include><model name="existing"/></world></sdf>')
        before = ET.tostring(original)
        result = append_to_world(original, self.packing)
        self.assertEqual(ET.tostring(original), before)
        self.assertEqual(result.find('world').get('name'), 'custom_world')
        self.assertEqual(result.findtext('world/include/uri'), 'model://robot')
        self.assertEqual(len(result.findall('world/model')), 101)
        with self.assertRaisesRegex(ValueError, 'conflict'):
            append_to_world(result, self.packing)

    def test_explicit_world_selection_and_include_name_conflict(self):
        root = ET.fromstring('<sdf version="1.8"><world name="one"/><world name="two"/></sdf>')
        with self.assertRaises(ValueError):
            append_to_world(root, self.packing)
        result = append_to_world(root, self.packing, world_name='two')
        self.assertEqual(len(result.findall("world[@name='one']/model")), 0)
        self.assertEqual(len(result.findall("world[@name='two']/model")), 100)
        root = ET.fromstring('<sdf><world name="w"><include><name>inventory_ball_001</name>'
                             '<uri>model://something</uri></include></world></sdf>')
        with self.assertRaises(ValueError):
            append_to_world(root, self.packing)

    def test_cli_never_overwrites_input_or_implicit_output(self):
        tool = Path(__file__).resolve().parents[1] / 'tools/generate_ball_inventory.py'
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'world.sdf'
            target = Path(directory) / 'populated.sdf'
            source.write_text('<sdf version="1.8"><world name="keep_me"/></sdf>')
            command = [sys.executable, str(tool), '--bounds', '0', '0', '1', '.3', '.3', '1.3',
                       '--input-world', str(source), '--output', str(target)]
            run = subprocess.run(command, capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stderr)
            self.assertEqual(ET.parse(target).find('world').get('name'), 'keep_me')
            self.assertEqual(len(ET.parse(target).findall('world/model')), 100)
            self.assertNotEqual(subprocess.run(command, capture_output=True).returncode, 0)
            command[-1] = str(source)
            self.assertNotEqual(subprocess.run(command + ['--force'], capture_output=True).returncode, 0)
            self.assertEqual(len(ET.parse(source).findall('world/model')), 0)


class LedgerTests(unittest.TestCase):
    def setUp(self):
        self.names = tuple(f'ball_{i:03}' for i in range(100))
        self.ledger = BallLedger(self.names)

    def test_conserves_all_identities_through_observed_transit(self):
        for i, name in enumerate(self.names):
            for j, phase in enumerate((Phase.HOPPER, Phase.FEEDER, Phase.PIPE,
                                       Phase.HEAD, Phase.FLIGHT, Phase.COLLECTED)):
                self.assertTrue(self.ledger.observe(name, phase, i + j * 0.1))
                self.assertEqual(sum(self.ledger.counts().values()), 100)
        self.assertEqual(self.ledger.counts()['collected'], 100)
        self.assertTrue(self.ledger.audit(self.names)['all_accounted_for'])
        snapshot = self.ledger.snapshot()
        snapshot.clear()
        self.assertEqual(len(self.ledger.names), 100)

    def test_exposes_missing_extra_and_duplicate_without_replacing_balls(self):
        observed = self.names[1:] + ('unexpected_ball', self.names[1])
        audit = self.ledger.audit(observed)
        self.assertFalse(audit['all_accounted_for'])
        self.assertEqual(audit['missing'], [self.names[0]])
        self.assertEqual(audit['unexpected'], ['unexpected_ball'])
        self.assertEqual(audit['duplicates'], [self.names[1]])
        self.assertEqual(self.ledger.counts()['initialized'], 100)

    def test_invalid_observations_do_not_fabricate_counts(self):
        with self.assertRaises(KeyError):
            self.ledger.observe('new_ball', Phase.HEAD, 0)
        with self.assertRaises(ValueError):
            self.ledger.observe(self.names[0], 'imaginary_phase', 0)
        self.ledger.observe(self.names[0], Phase.HOPPER, 2)
        with self.assertRaises(ValueError):
            self.ledger.observe(self.names[0], Phase.FLIGHT, 1)
        with self.assertRaises(ValueError):
            self.ledger.observe(self.names[0], Phase.FLIGHT, float('nan'))
        self.assertEqual(self.ledger.counts()['hopper'], 1)
        self.ledger.reset()
        self.assertEqual(self.ledger.names, self.names)
        self.assertEqual(self.ledger.counts()['initialized'], 100)
        self.ledger.observe(self.names[0], Phase.HOPPER, 0)


if __name__ == '__main__':
    unittest.main()
