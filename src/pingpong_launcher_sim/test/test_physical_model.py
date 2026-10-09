"""Generated physical-world invariants; no runtime feeding claims."""
import itertools
import math
from pathlib import Path
import struct
import unittest
import xml.etree.ElementTree as ET

PACKAGE = Path(__file__).resolve().parents[1]
MODEL = PACKAGE / 'models/pingpong_r10'


class PhysicalModelInvariantTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.world = ET.parse(PACKAGE / 'worlds/physical_100.sdf').getroot().find('world')
        cls.model = ET.parse(MODEL / 'model.sdf').getroot().find('model')

    def test_exactly_100_unique_persistent_ball_models(self):
        balls = self.world.findall("model/link[@name='ball_link']/..")
        self.assertEqual({b.get('name') for b in balls},
                         {f'inventory_ball_{i:03d}' for i in range(1, 101)})
        self.assertEqual(len(balls), 100)
        for ball in balls:
            self.assertNotEqual(ball.findtext('static'), 'true')
            self.assertEqual(ball.findall('.//plugin'), [])
            self.assertAlmostEqual(float(ball.findtext('link/inertial/mass')), .0027)
            self.assertAlmostEqual(float(ball.findtext('link/collision/geometry/sphere/radius')), .020)

    def test_initial_balls_do_not_overlap(self):
        positions = [tuple(map(float, b.findtext('pose').split()[:3]))
                     for b in self.world.findall('model')
                     if b.get('name', '').startswith('inventory_ball_')]
        self.assertEqual(len(positions), 100)
        minimum = min(math.dist(a, b) for a, b in itertools.combinations(positions, 2))
        self.assertGreaterEqual(minimum, .040)

    def test_physical_mode_has_no_legacy_muzzle_generator(self):
        all_plugins = list(self.model.findall('plugin')) + list(self.world.findall('plugin'))
        names = ' '.join(str(p.attrib) for p in all_plugins)
        self.assertIn('PhysicalBallMonitor', names)
        self.assertNotIn('libpingpong_ball_launcher.so', names)
        self.assertEqual(len(self.world.findall('include')), 1)
        self.assertEqual(self.world.findtext('include/uri'), 'model://pingpong_r10')

    def test_collisions_reference_existing_closed_positive_volume_meshes(self):
        checked = set()
        for uri in self.model.findall('.//collision/geometry/mesh/uri'):
            prefix = 'model://pingpong_r10/'
            self.assertTrue(uri.text.startswith(prefix), uri.text)
            path = MODEL / uri.text[len(prefix):]
            if path in checked:
                continue
            checked.add(path)
            raw = path.read_bytes()
            count = struct.unpack_from('<I', raw, 80)[0]
            self.assertEqual(len(raw), 84 + 50 * count, str(path))
            edges = {}
            volume6 = 0.0
            for i in range(count):
                values = struct.unpack_from('<12f', raw, 84 + 50 * i)
                vertices = [tuple(values[j:j + 3]) for j in (3, 6, 9)]
                a, b, c = vertices
                self.assertTrue(all(math.isfinite(x) for v in vertices for x in v))
                cross = (b[1]*c[2]-b[2]*c[1], b[2]*c[0]-b[0]*c[2], b[0]*c[1]-b[1]*c[0])
                volume6 += sum(x*y for x, y in zip(a, cross))
                for start, end in zip(vertices, vertices[1:] + vertices[:1]):
                    key = tuple(sorted((start, end)))
                    edges[key] = edges.get(key, 0) + 1
            self.assertGreater(abs(volume6), 1e-15, str(path))
            self.assertTrue(all(n == 2 for n in edges.values()), f'Open/non-manifold prism: {path.name}')
        self.assertGreater(len(checked), 0)


if __name__ == '__main__':
    unittest.main()

class IntegratedHeadMeterTests(unittest.TestCase):
    def setUp(self):
        self.model=ET.parse(MODEL/'model.sdf').getroot().find('model')

    def test_roller_contact_geometry_and_joint_parentage(self):
        for side,y in [('roller',.025),('idler',-.025)]:
            link=self.model.find(f"link[@name='head_meter_{side}_link']")
            xyz=list(map(float,link.findtext('pose').split()[:3]))
            self.assertEqual(xyz,[.192,y,.186])
            self.assertAlmostEqual(float(link.findtext('collision/geometry/cylinder/radius')),.006)
            self.assertAlmostEqual(float(link.findtext('collision/geometry/cylinder/length')),.006)
        slide=self.model.find("joint[@name='head_meter_idler_slide']")
        self.assertEqual(slide.get('type'),'prismatic')
        self.assertEqual(slide.findtext('parent'),'head_link')
        self.assertEqual(slide.findtext('axis/xyz'),'0 -1 0')
        self.assertAlmostEqual(float(slide.findtext('axis/limit/upper')),.002)
        self.assertEqual(self.model.findtext("joint[@name='head_meter_idler_joint']/parent"),'head_meter_idler_carriage_link')

    def test_drain_is_a_separate_installed_hollow_part(self):
        cover=self.model.find("link[@name='drain_cover_link']")
        self.assertGreater(len(cover.findall('collision')),0)
        self.assertEqual(self.model.find("joint[@name='drain_cover_installed_joint']").get('type'),'fixed')
        self.assertIsNone(self.model.find("link[@name='base_link']/collision[@name='case_left']"))
        self.assertIsNotNone(self.model.find("link[@name='drain_service_panel_link']/collision[@name='service_panel']"))

    def test_every_cad_visual_is_packaged_and_uses_millimetre_scale(self):
        import json
        manifest=json.loads((MODEL/'meshes/cad_meter/manifest.json').read_text())
        self.assertGreaterEqual(len(manifest['records']),68)
        for rec in manifest['records']:
            self.assertTrue((MODEL/'meshes/cad_meter'/rec['file']).exists())
            v=self.model.find(f"link[@name='{rec['link']}']/visual[@name='{rec['file'][:-4]}']")
            self.assertIsNotNone(v)
            self.assertEqual([float(x) for x in v.findtext('geometry/mesh/scale').split()],[.001]*3)
