import pathlib
import sys
import unittest

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'ros2_ws/src/arm_vision'))
from arm_vision.yolo_onnx import decode, letterbox


def raw_output(rows, classes):
    """Build a (1, 4 + classes, anchors) YOLOv8 tensor from (cx, cy, w, h, class, score)."""
    out = np.zeros((1, 4 + classes, len(rows)), dtype=np.float32)
    for i, (cx, cy, w, h, cls, score) in enumerate(rows):
        out[0, :4, i] = (cx, cy, w, h)
        out[0, 4 + cls, i] = score
    return out


class YoloOnnxTests(unittest.TestCase):
    def test_letterbox_pads_a_4_3_frame_vertically(self):
        image, scale, pad_x, pad_y = letterbox(np.zeros((240, 320, 3), np.uint8), 320)
        self.assertEqual(image.shape, (320, 320, 3))
        self.assertEqual((scale, pad_x, pad_y), (1.0, 0, 40))
        self.assertTrue((image[0] == 114).all())      # padding row is grey

    def test_box_maps_back_to_frame_pixels(self):
        # A 320x240 frame letterboxed into 320: 40 px of padding top and bottom.
        # An object at frame (100, 60)-(160, 120) sits at input (100, 100)-(160, 160).
        out = raw_output([(130, 130, 60, 60, 1, 0.9)], classes=2)
        found = decode(out, 1.0, 0, 40, 320, 240, confidence=0.5)
        self.assertEqual(len(found), 1)
        cls, score, box = found[0]
        self.assertEqual(cls, 1)
        self.assertAlmostEqual(score, 0.9, places=5)
        self.assertEqual(box, [100, 60, 60, 60])

    def test_below_confidence_is_dropped(self):
        out = raw_output([(130, 130, 60, 60, 0, 0.2)], classes=1)
        self.assertEqual(decode(out, 1.0, 0, 40, 320, 240, confidence=0.5), [])

    def test_overlapping_duplicates_collapse_to_one(self):
        out = raw_output([(130, 130, 60, 60, 0, 0.9), (132, 131, 60, 60, 0, 0.8)], classes=1)
        found = decode(out, 1.0, 0, 40, 320, 240, confidence=0.5)
        self.assertEqual(len(found), 1)
        self.assertAlmostEqual(found[0][1], 0.9, places=5)

    def test_scale_is_undone_for_a_larger_frame(self):
        # 640x480 into 320: scale 0.5, padding 40. Input box (100,100)-(160,160)
        # is frame (200, 120)-(320, 240).
        out = raw_output([(130, 130, 60, 60, 0, 0.9)], classes=1)
        _, _, box = decode(out, 0.5, 0, 40, 640, 480, confidence=0.5)[0]
        self.assertEqual(box, [200, 120, 120, 120])


if __name__ == '__main__':
    unittest.main()
