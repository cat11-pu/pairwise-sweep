"""Behaviour tests for the sweep collision helpers."""

import math
import unittest

from sweep.core import (
    AABB,
    aabb_from_ltrb,
    move_and_collide,
    substep_count,
    sweep_aabb,
)


class GeometryTests(unittest.TestCase):
    def test_invalid_extents_are_rejected(self):
        with self.assertRaises(ValueError):
            AABB(0.0, 0.0, 0.0, 1.0)
        with self.assertRaises(ValueError):
            AABB(0.0, 0.0, 1.0, -2.0)
        with self.assertRaises(ValueError):
            aabb_from_ltrb(1.0, 0.0, 1.0, 4.0)
        with self.assertRaises(ValueError):
            aabb_from_ltrb(3.0, 0.0, 1.0, 4.0)
        rect = aabb_from_ltrb(-1.0, -2.0, 3.0, 2.0)
        self.assertAlmostEqual(rect.cx, 1.0)
        self.assertAlmostEqual(rect.cy, 0.0)
        self.assertAlmostEqual(rect.hw, 2.0)
        self.assertAlmostEqual(rect.hh, 2.0)

    def test_touching_boxes_do_not_overlap(self):
        platform = AABB(0.0, 0.0, 6.0, 1.0)
        standing = AABB(0.0, 2.0, 1.0, 1.0)
        self.assertFalse(standing.overlaps(platform))
        self.assertFalse(platform.overlaps(standing))
        wall = AABB(4.0, 0.0, 1.0, 3.0)
        beside = AABB(2.0, 0.0, 1.0, 1.0)
        self.assertFalse(beside.overlaps(wall))
        self.assertFalse(wall.overlaps(beside))
        sunk = AABB(0.0, 1.9, 1.0, 1.0)
        self.assertTrue(sunk.overlaps(platform))
        self.assertTrue(platform.overlaps(sunk))

    def test_expanded_grows_on_every_side(self):
        base = AABB(1.0, 2.0, 1.0, 0.5)
        grown = base.expanded(0.25)
        self.assertAlmostEqual(grown.cx, 1.0)
        self.assertAlmostEqual(grown.cy, 2.0)
        self.assertAlmostEqual(grown.hw, 1.25)
        self.assertAlmostEqual(grown.hh, 0.75)


class SweepTests(unittest.TestCase):
    def test_sweep_into_negative_x_reaches_the_wall(self):
        body = AABB(5.0, 0.0, 1.0, 1.0)
        wall = AABB(1.0, 0.0, 1.0, 6.0)
        hit = sweep_aabb(body, -10.0, 0.0, [wall])
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.t, 0.2, places=9)
        self.assertAlmostEqual(hit.nx, 1.0, places=9)
        self.assertAlmostEqual(hit.ny, 0.0, places=9)
        self.assertAlmostEqual(hit.box.left, wall.right, places=9)

    def test_sweep_from_exact_contact_blocks_movement(self):
        wall = AABB(2.0, 0.0, 1.0, 4.0)
        body = AABB(0.0, 0.0, 1.0, 1.0)
        self.assertAlmostEqual(body.right, wall.left, places=9)
        hit = sweep_aabb(body, 1.0, 0.0, [wall])
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.t, 0.0, places=9)
        self.assertAlmostEqual(hit.nx, -1.0, places=9)
        self.assertAlmostEqual(hit.ny, 0.0, places=9)

    def test_diagonal_hit_normal_follows_the_hit_face(self):
        body = AABB(0.0, 0.0, 1.0, 1.0)
        block = AABB(7.0, 8.0, 1.0, 3.0)
        hit = sweep_aabb(body, 10.0, 20.0, [block])
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.t, 0.5, places=9)
        self.assertAlmostEqual(hit.nx, -1.0, places=9)
        self.assertAlmostEqual(hit.ny, 0.0, places=9)
        self.assertAlmostEqual(hit.box.right, block.left, places=9)

    def test_corner_hit_normal_is_unit_length(self):
        body = AABB(0.0, 0.0, 1.0, 1.0)
        block = AABB(7.0, 7.0, 1.0, 1.0)
        hit = sweep_aabb(body, 10.0, 10.0, [block])
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.t, 0.5, places=9)
        self.assertAlmostEqual(math.hypot(hit.nx, hit.ny), 1.0, places=9)
        self.assertLess(hit.nx, 0.0)
        self.assertLess(hit.ny, 0.0)
        self.assertAlmostEqual(hit.box.right, block.left, places=9)
        self.assertAlmostEqual(hit.box.top, block.bottom, places=9)


class MovementTests(unittest.TestCase):
    def test_high_speed_fall_rests_on_thin_slab(self):
        slab = AABB(0.0, -1.0, 15.0, 1.0)
        body = AABB(0.0, 5.0, 1.0, 1.0)
        steps = substep_count(body, 0.0, -25.0, [slab], max_step=10.0)
        thinnest = min(10.0, body.width, body.height, slab.width, slab.height)
        self.assertLessEqual(25.0 / steps, thinnest + 1e-9)
        landed, hits = move_and_collide(body, 0.0, -25.0, [slab], max_step=10.0)
        self.assertGreaterEqual(landed.bottom, slab.top - 1e-9)
        self.assertAlmostEqual(landed.bottom, slab.top, places=6)
        self.assertAlmostEqual(landed.cx, body.cx, places=9)
        self.assertFalse(landed.overlaps(slab))
        self.assertGreaterEqual(len(hits), 1)

    def test_body_overlapping_a_surface_is_pushed_clear(self):
        ground = AABB(0.0, -1.0, 10.0, 1.0)
        body = AABB(0.0, 0.75, 1.0, 1.0)
        self.assertTrue(body.overlaps(ground))
        pushed, hits = move_and_collide(body, 0.0, 0.125, [ground], max_step=1.0)
        self.assertGreaterEqual(pushed.bottom, ground.top - 1e-9)
        self.assertAlmostEqual(pushed.bottom, ground.top, places=6)
        self.assertFalse(pushed.overlaps(ground))
        self.assertGreaterEqual(len(hits), 1)


if __name__ == "__main__":
    unittest.main()
