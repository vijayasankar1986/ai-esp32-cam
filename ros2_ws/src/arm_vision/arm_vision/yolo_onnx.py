"""YOLOv8 ONNX models from vision-platform, run through OpenCV DNN.

vision-platform (autotransport/vision-platform) fine-tunes YOLOv8n on frames
labelled in its console and exports `model.onnx` plus `labels.json`. OpenCV
reads the ONNX directly, so the Pi needs neither torch nor ultralytics.

Kept free of ROS so the maths can be tested on any machine.
"""

import json
import os

import numpy as np

import cv2


def letterbox(frame, size):
    """Fit the frame into size x size without distortion, padding with grey.

    The model was trained on letterboxed images, so a plain resize would
    stretch a 4:3 frame and shift every box. Returns the padded image and the
    scale and offsets needed to map boxes back.
    """
    height, width = frame.shape[:2]
    scale = size / max(height, width)
    new_w, new_h = round(width * scale), round(height * scale)
    pad_x, pad_y = (size - new_w) // 2, (size - new_h) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = cv2.resize(frame, (new_w, new_h))
    return canvas, scale, pad_x, pad_y


def decode(output, scale, pad_x, pad_y, width, height, confidence, nms=0.45):
    """Turn raw YOLOv8 output into (class_id, score, [x, y, w, h]) in frame pixels.

    YOLOv8 emits (1, 4 + classes, anchors): centre x, centre y, width and
    height in model-input pixels, then one score per class, with no separate
    objectness term.
    """
    rows = np.squeeze(output, 0).T
    scores = rows[:, 4:]
    class_ids = np.argmax(scores, axis=1)
    best = scores[np.arange(len(rows)), class_ids]
    keep = best >= confidence
    if not np.any(keep):
        return []
    rows, class_ids, best = rows[keep], class_ids[keep], best[keep]

    boxes = []
    for cx, cy, w, h in rows[:, :4]:
        x = (cx - w / 2 - pad_x) / scale
        y = (cy - h / 2 - pad_y) / scale
        x0, y0 = max(0.0, x), max(0.0, y)
        x1, y1 = min(float(width), x + w / scale), min(float(height), y + h / scale)
        boxes.append([int(x0), int(y0), int(max(0.0, x1 - x0)), int(max(0.0, y1 - y0))])

    picked = cv2.dnn.NMSBoxes(boxes, best.astype(float).tolist(), confidence, nms)
    return [(int(class_ids[i]), float(best[i]), boxes[i])
            for i in np.array(picked).flatten()]


class YoloOnnx:
    """A vision-platform model: model.onnx with labels.json beside it."""

    def __init__(self, path):
        labels_path = os.path.join(os.path.dirname(path), 'labels.json')
        with open(labels_path) as handle:
            labels = json.load(handle)
        self.names = labels['names']
        self.size = int(labels.get('imgsz', 320))
        self.net = cv2.dnn.readNetFromONNX(path)

    def detect(self, frame, confidence):
        height, width = frame.shape[:2]
        image, scale, pad_x, pad_y = letterbox(frame, self.size)
        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (self.size, self.size),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        found = decode(self.net.forward(), scale, pad_x, pad_y, width, height, confidence)
        return [(self.names[c], s, box) for c, s, box in found]
