import math
import unittest

try:
    import torch
    from unisim.utils.math import quat_apply
except ImportError:
    torch = None


@unittest.skipIf(torch is None, 'Requires scene runtime Torch and UniSim')
class InsertGeometryTests(unittest.TestCase):
    def setUp(self):
        from integrations.unilab.insert_geometry import geometry_checks
        self.evaluate = geometry_checks
        self.rack = torch.tensor([[x, y, z] for x in (-.1, .1)
                                  for y in (-.05, .05) for z in (-.03, .03)], dtype=torch.float64)
        self.tube = torch.tensor([[x, y, z] for x in (-.01, .01)
                                  for y in (0., .14) for z in (-.01, .01)], dtype=torch.float64)
        self.pos = torch.zeros((1, 4, 3), dtype=torch.float64)
        self.pos[:, :3, 2] = -.02
        self.quat = torch.tensor([[[math.sqrt(.5), math.sqrt(.5), 0, 0]] * 3
                                  + [[1., 0, 0, 0]]], dtype=torch.float64)

    def check(self):
        return self.evaluate(self.pos, self.quat, self.tube, self.rack)

    def test_upright_below_rim_matches_three_source_geometry_conditions(self):
        self.assertTrue(self.check().all())

    def test_inside_is_strict_xy_and_above_bottom(self):
        self.pos[0, 0, 0] = .1
        self.pos[0, 1, 2] = -.03
        checks = self.check()
        self.assertFalse(checks[0, 0, 1])
        self.assertFalse(checks[0, 1, 1])
        self.assertTrue(checks[0, 2, 1])

    def test_laying_tube_fails_upright_and_high_tube_fails_depth(self):
        self.quat[0, 0] = torch.tensor([1., 0, 0, 0])
        self.pos[0, 1, 2] = .04
        checks = self.check()
        self.assertFalse(checks[0, 0, 2])
        self.assertFalse(checks[0, 1, 0])

    def test_rack_yaw_and_translation_preserve_inside(self):
        from unisim.utils.math import quat_mul
        yaw = torch.tensor([math.cos(.4), 0., 0., math.sin(.4)], dtype=torch.float64)
        self.pos = quat_apply(yaw, self.pos) + torch.tensor([.3, -.4, .8])
        self.quat = quat_mul(yaw, self.quat)
        self.assertTrue(self.check().all())

    def test_inside_does_not_mean_inside_a_hole(self):
        # Source checks the rack bounding polygon, not individual apertures.
        self.pos[0, 0, 0] = .018
        self.assertTrue(self.check()[0, 0].all())

    def test_nonfinite_and_nonunit_inputs_fail_closed(self):
        self.pos[0, 0, 0] = float('nan')
        with self.assertRaises(ValueError):
            self.check()
        self.pos[0, 0, 0] = 0
        self.quat[0, 0] *= 2
        with self.assertRaises(ValueError):
            self.check()

    def test_random_tilted_boxes_against_source_polygon_formula(self):
        try:
            import numpy as np
            from shapely.geometry import MultiPoint, Point
            from transforms3d.quaternions import quat2mat
        except ImportError:
            self.skipTest('Optional source-reference test needs shapely and transforms3d')
        rng = np.random.default_rng(42)
        positions = rng.uniform(-.12, .12, (128, 4, 3))
        quats = rng.normal(size=(128, 4, 4))
        quats /= np.linalg.norm(quats, axis=-1, keepdims=True)
        actual = self.evaluate(torch.tensor(positions), torch.tensor(quats), self.tube, self.rack).numpy()
        expected = np.zeros_like(actual)
        for row in range(128):
            rack = self.rack.numpy() @ quat2mat(quats[row, 3]).T + positions[row, 3]
            polygon = MultiPoint(rack[:, :2]).convex_hull.buffer(0)
            for tube in range(3):
                rotation = quat2mat(quats[row, tube])
                box = self.tube.numpy() @ rotation.T + positions[row, tube]
                expected[row, tube] = [
                    rack[:, 2].max() - box[:, 2].min() > .045,
                    polygon.contains(Point(positions[row, tube, :2])) and positions[row, tube, 2] > rack[:, 2].min(),
                    np.degrees(np.arccos(np.clip(rotation[2, 1], -1, 1))) < 30,
                ]
        np.testing.assert_array_equal(actual, expected)


if __name__ == '__main__':
    unittest.main()
