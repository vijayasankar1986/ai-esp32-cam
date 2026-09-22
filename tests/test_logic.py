import pathlib
import sys
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/arm_poc'))
from arm_poc.logic import COLOR_RANGES, DetectionGate, color_ranges, move_command


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


if __name__ == '__main__':
    unittest.main()
