import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/arm_poc'))
from arm_poc.logic import (COLOR_RANGES, DetectionGate, clean_reply, color_ranges,
                           fit_pixel_to_joints, move_command, pose_from_pixel)


class LogicTests(unittest.TestCase):
    def test_one_trigger_until_stable_removal(self):
        gate = DetectionGate(3)
        self.assertEqual([gate.update(True) for _ in range(5)], [False, False, True, False, False])
        gate.update(False)
        self.assertFalse(gate.update(True))
        for _ in range(3):
            gate.update(False)
        self.assertEqual([gate.update(True) for _ in range(3)], [False, False, True])

    def test_noise_does_not_trigger(self):
        gate = DetectionGate(3)
        for value in [True, False] * 20:
            self.assertFalse(gate.update(value))

    def test_command(self):
        self.assertEqual(move_command([80, 90, 95, 100], [80]*4, [100]*4), b'MOVE 80 90 95 100\n')

    def test_reject_bad_commands(self):
        for pose in ([90]*3, [79]*4, [101]*4, [float('nan')]*4, [90.5]*4):
            with self.assertRaises(ValueError):
                move_command(pose, [80]*4, [100]*4)


class ReplyTests(unittest.TestCase):
    def test_servo_noise_before_ok_is_dropped(self):
        # Captured from the Pi: servo motion glitched the line ahead of OK.
        self.assertEqual(clean_reply(b'\xff' * 64 + b'OK\r\n'), b'OK')

    def test_clean_reply_unchanged(self):
        self.assertEqual(clean_reply(b'READY\r\n'), b'READY')

    def test_real_failures_still_differ(self):
        self.assertNotEqual(clean_reply(b'\xffets Jul 29 2019\r\n'), b'OK')
        self.assertNotEqual(clean_reply(b'ERR limits\n'), b'OK')
        self.assertEqual(clean_reply(b''), b'')


class ColorTests(unittest.TestCase):
    def test_known_colors_resolve(self):
        for name in COLOR_RANGES:
            self.assertTrue(color_ranges(name.upper().center(len(name) + 2)))

    def test_red_wraps_the_hue_circle(self):
        self.assertEqual(len(color_ranges('red')), 2)

    def test_reject_unknown_color(self):
        for bad in ('magenta', '', 'rd', None, 7):
            with self.assertRaises(ValueError):
                color_ranges(bad)

    def test_bounds_are_valid_hsv(self):
        for name, ranges in COLOR_RANGES.items():
            for low, high in ranges:
                self.assertEqual((len(low), len(high)), (3, 3), name)
                self.assertLess(low[0], high[0], name)
                for i, ceiling in enumerate((179, 255, 255)):
                    self.assertGreaterEqual(low[i], 0, name)
                    self.assertLessEqual(high[i], ceiling, name)


SQUARE = [(0.0, 0.0, [80, 80, 80, 80]), (1.0, 0.0, [100, 80, 90, 80]),
          (0.0, 1.0, [80, 100, 80, 90]), (1.0, 1.0, [100, 100, 90, 90])]


class CalibrationTests(unittest.TestCase):
    def test_corners_reproduce_their_samples(self):
        model = fit_pixel_to_joints(SQUARE)
        for u, v, angles in SQUARE:
            self.assertEqual(pose_from_pixel(model, u, v, [80]*4, [100]*4), angles)

    def test_centre_interpolates(self):
        model = fit_pixel_to_joints(SQUARE)
        self.assertEqual(pose_from_pixel(model, 0.5, 0.5, [80]*4, [100]*4), [90, 90, 85, 85])

    def test_extrapolation_is_clamped_to_limits(self):
        model = fit_pixel_to_joints(SQUARE)
        for u, v in ((5.0, 5.0), (-4.0, -4.0)):
            for angle, lo, hi in zip(pose_from_pixel(model, u, v, [80]*4, [100]*4),
                                     [80]*4, [100]*4):
                self.assertTrue(lo <= angle <= hi)

    def test_reject_degenerate_calibration(self):
        collinear = [(0.0, 0.0, [90]*4), (0.5, 0.5, [90]*4), (1.0, 1.0, [90]*4)]
        for bad in (SQUARE[:2], collinear):
            with self.assertRaises(ValueError):
                fit_pixel_to_joints(bad)

    def test_reject_out_of_range_or_malformed_points(self):
        for bad in ([(0.0, 0.0, [90]*3)] + SQUARE[1:], [(1.5, 0.0, [90]*4)] + SQUARE[1:]):
            with self.assertRaises(ValueError):
                fit_pixel_to_joints(bad)


if __name__ == '__main__':
    unittest.main()
