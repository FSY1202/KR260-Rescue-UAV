"""Route/scan regressions using the actual two supplied reference images."""
import json
from pathlib import Path
import unittest

import numpy as np
from generate_point_cloud import generate
from map_display import AnimationClock, load_trajectory
from point_cloud_map import Polyline, PointCloudMap, transform_reference

DATA = Path(__file__).resolve().parents[1]/'data'


class PresetTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        points, duration = load_trajectory(DATA/'trajectory.json')
        cls.path = Polyline(points, duration)

    def test_reference_alignment_and_reproducibility(self):
        a, route = generate(DATA/'reference_map.png', DATA/'reference_route.png')
        b, _ = generate(DATA/'reference_map.png', DATA/'reference_route.png')
        for key in a:
            np.testing.assert_array_equal(a[key], b[key])
        with np.load(DATA/'map_cloud.npz') as saved:
            for key in a:
                np.testing.assert_array_equal(a[key], saved[key])
        meta = json.loads(str(a['metadata'].item()))
        self.assertEqual(len(meta['cartons']), 8)
        self.assertEqual(len(meta['gate_centers']), 2)
        self.assertEqual(len(a['points']), 12000)
        np.testing.assert_allclose(self.path.points, route['points'])

    def test_bottom_start_and_gate_centers(self):
        with np.load(DATA/'map_cloud.npz') as saved:
            meta = json.loads(str(saved['metadata'].item()))
        gates = transform_reference(meta['gate_centers'])
        p = self.path.points
        self.assertGreater(p[0, 1], p[1, 1])
        self.assertEqual(p[0, 1], p[-1, 1])
        self.assertLess(p[0, 0], p[-1, 0])
        self.assertAlmostEqual(p[4, 0], gates[0, 0])
        self.assertGreater(p[4, 1], gates[0, 1])
        self.assertLess(p[5, 1], gates[0, 1])
        self.assertAlmostEqual(p[6, 0], gates[1, 0])
        self.assertLess(p[6, 1], gates[1, 1])
        self.assertGreater(p[7, 1], gates[1, 1])

    def test_arc_length_speed_and_corners(self):
        path = self.path
        for i, length in enumerate(path.lengths):
            t0, t1 = path.distances[i:i+2]/path.total*path.duration
            a, b = np.array(path.at(t0+.1)[:2]), np.array(path.at(t1-.1)[:2])
            self.assertAlmostEqual(np.linalg.norm(b-a)/(t1-t0-.2), path.total/60)
            trail = path.trail(t1)
            np.testing.assert_allclose(trail[:i+1], np.rint(path.points[:i+1]))
        np.testing.assert_allclose(path.at(100)[:2], path.points[-1])
        np.testing.assert_allclose(path.at(-10)[:2], path.points[0])

    def test_scan_accumulates_and_hidden_clock_catches_up(self):
        cloud = PointCloudMap(DATA/'map_cloud.npz', self.path)
        frame = np.empty((600, 800, 4), np.uint8)
        counts = [cloud.draw(frame, t, self.path) for t in (0, 10, 20, 30, 45, 60)]
        self.assertEqual(counts, sorted(counts))
        self.assertGreater(counts[0], 0)
        self.assertLess(counts[0], 12000)
        self.assertEqual(counts[-1], 12000)
        final = frame.copy()
        clock = AnimationClock(60)
        clock.start(100)
        cloud.draw(frame, clock.elapsed(110), self.path)
        cloud.draw(frame, clock.elapsed(165), self.path)
        np.testing.assert_array_equal(frame, final)
        self.assertEqual(clock.elapsed(190), 60)
        clock.start(200)
        self.assertEqual(clock.elapsed(201), 60)


if __name__ == '__main__':
    unittest.main()
