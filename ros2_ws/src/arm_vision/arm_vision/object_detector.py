"""Name the object instead of guessing it from its colour.

The existing detector in arm_poc thresholds a hue and takes the largest blob.
That is fast, deterministic and completely blind: a red mug, a red pen and a
red sleeve are the same object to it. This node runs SSD MobileNet v3 through
OpenCV's DNN module instead, so a detection carries a class name and the arm
can be told to fetch the cup rather than the nearest red thing.

It publishes the same message shape arm_poc does -- normalised u, v and a size
fraction on a PointStamped -- so arm_moveit_bridge's vision_pick consumes
either source without knowing which is running. On a different topic, though:
two publishers on /vision/target_point would interleave and the consumer could
not tell whose pixel it had.

No TensorFlow here. OpenCV reads the frozen graph directly, which is why this
adds no runtime to a Pi that already depends on cv2. Measured on the Pi 4:
about 0.33 s per 320x240 frame, against a camera delivering 2.8 a second, so
the node drops frames rather than queueing them.

What it does NOT do is tell you where the object is in space. A box centre is
a pixel, exactly like the colour centroid was, and turning a pixel into a
point still needs the camera pose and the table-plane assumption in
vision_pick. Better labels, same geometry problem.
"""

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

from arm_vision.coco_labels import GRASPABLE, name
from arm_vision.yolo_onnx import YoloOnnx

#: SSD MobileNet v3 was trained at this size with this normalisation. These
#: are properties of the model, not preferences.
INPUT = 320
SCALE = 1.0 / 127.5
MEAN = (127.5, 127.5, 127.5)


class ObjectDetector(Node):

    def __init__(self):
        super().__init__('arm_vision_detector')

        models = os.path.expanduser('~/models')
        self.declare_parameter('graph', os.path.join(
            models, 'ssd_mobilenet_v3.pbtxt'))
        self.declare_parameter('weights', os.path.join(
            models, 'ssd_mobilenet_v3_large_coco_2020_01_14',
            'frozen_inference_graph.pb'))
        # Empty means "whatever it is most confident about, if the arm could
        # plausibly pick it up". Name a class to hunt for one thing.
        self.declare_parameter('target_class', '')
        # On this 320x240 Wi-Fi feed a genuine cup scores about 0.30-0.35,
        # so a textbook 0.5 detects nothing at all. Lower means more false
        # positives; a higher camera resolution would serve better than
        # tuning this number.
        self.declare_parameter('confidence', 0.35)
        self.declare_parameter('graspable_only', True)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('min_period', 0.4)
        # A model.onnx trained in vision-platform, with labels.json beside it.
        # Empty keeps the stock COCO SSD. See tools/fetch_platform_model.py.
        self.declare_parameter('model', '')

        g = lambda n: self.get_parameter(n).value
        graph, weights = g('graph'), g('weights')
        self.want = g('target_class').strip().lower()
        self.confidence = g('confidence')
        self.graspable_only = g('graspable_only')
        self.annotate = g('publish_annotated')
        self.min_period = g('min_period')
        model = os.path.expanduser(g('model').strip())

        if model:
            if not os.path.exists(model):
                self.get_logger().error(
                    f'Model file missing: {model}\n'
                    'Run tools/fetch_platform_model.py to download it.')
                raise SystemExit(1)
            self.yolo = YoloOnnx(model)
            # Every class in a trained model was put there on purpose, so the
            # COCO "graspable" filter does not apply to it.
            self.graspable_only = False
            self.kind = f'vision-platform YOLOv8 ({", ".join(self.yolo.names)})'
        else:
            self.yolo = None
            self.kind = 'SSD MobileNet v3 via OpenCV DNN'
            self.load_ssd(graph, weights)

        self.bridge = CvBridge()
        self.last = 0.0
        self.busy = False

        self.point = self.create_publisher(PointStamped, '/vision/object_point', 10)
        self.seen = self.create_publisher(Bool, '/vision/object_detected', 10)
        self.label = self.create_publisher(String, '/vision/object_label', 10)
        self.overlay = (self.create_publisher(Image, '/vision/annotated', 10)
                        if self.annotate else None)

        self.create_subscription(Image, '/camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)

        self.get_logger().info(
            f'Detector ready: {self.kind}, looking for '
            + (f'"{self.want}"' if self.want else
               'anything graspable' if self.graspable_only else 'anything'))

    def load_ssd(self, graph, weights):
        for path in (graph, weights):
            if not os.path.exists(path):
                self.get_logger().error(
                    f'Model file missing: {path}\n'
                    'Run tools/fetch_detection_model.sh to download it.')
                raise SystemExit(1)

        self.net = cv2.dnn_DetectionModel(graph, weights)
        self.net.setInputSize(INPUT, INPUT)
        self.net.setInputScale(SCALE)
        self.net.setInputMean(MEAN)
        self.net.setInputSwapRB(True)

    def on_image(self, message):
        # Inference costs a third of a second and frames arrive faster than
        # that. Skipping is the honest response: a queue would only serve
        # older and older pictures of the table.
        now = time.monotonic()
        if self.busy or now - self.last < self.min_period:
            return
        self.busy = True
        try:
            self.detect(message)
        except Exception as exc:                  # a bad frame must not kill the node
            self.get_logger().warn(f'Detection failed: {exc}')
        finally:
            self.last = time.monotonic()
            self.busy = False

    def detect(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, 'bgr8')
        height, width = frame.shape[:2]
        found = []
        if self.yolo is not None:
            found = self.yolo.detect(frame, float(self.confidence))
            ids = []
        else:
            ids, confs, boxes = self.net.detect(
                frame, confThreshold=float(self.confidence))
        if len(ids):
            for i, c, b in zip(np.array(ids).flatten(),
                               np.array(confs).flatten(), boxes):
                found.append((name(i), float(c), [int(v) for v in b]))

        pick = self.choose(found)
        self.seen.publish(Bool(data=pick is not None))

        if pick is not None:
            label, conf, (x, y, w, h) = pick
            point = PointStamped()
            point.header.stamp = message.header.stamp
            point.header.frame_id = 'camera_optical_frame'
            # Same contract as arm_poc: centre as a fraction of the frame,
            # and how much of the frame the object fills.
            point.point.x = (x + w / 2.0) / width
            point.point.y = (y + h / 2.0) / height
            point.point.z = (w * h) / float(width * height)
            self.point.publish(point)
            self.label.publish(String(data=label))

        if self.overlay is not None:
            self.overlay.publish(self.bridge.cv2_to_imgmsg(
                self.draw(frame, found, pick), 'bgr8'))

    def choose(self, found):
        """The one detection worth reaching for, or None."""
        if not found:
            return None
        if self.want:
            matches = [f for f in found if f[0].lower() == self.want]
        elif self.graspable_only:
            matches = [f for f in found if f[0] in GRASPABLE]
        else:
            matches = list(found)
        if not matches:
            return None
        return max(matches, key=lambda f: f[1])

    @staticmethod
    def draw(frame, found, pick):
        """Boxes for everything, and a brighter one for the chosen object."""
        out = frame.copy()
        for label, conf, (x, y, w, h) in found:
            chosen = pick is not None and (label, conf) == (pick[0], pick[1])
            colour = (120, 220, 60) if chosen else (150, 150, 150)
            cv2.rectangle(out, (x, y), (x + w, y + h), colour,
                          2 if chosen else 1)
            cv2.putText(out, f'{label} {conf:.2f}', (x, max(12, y - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, colour, 1, cv2.LINE_AA)
        return out


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ObjectDetector()
    except SystemExit:
        rclpy.shutdown()
        raise
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
