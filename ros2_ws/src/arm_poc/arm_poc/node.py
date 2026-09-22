import math
import threading
import time

import cv2
import numpy as np
import requests
import serial
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from .logic import (DetectionGate, color_ranges, fit_pixel_to_joints,
                    move_command, pose_from_pixel)


class CameraReader:
    """Bounded JPEG extraction for snapshot and MJPEG HTTP responses."""

    def __init__(self, url):
        self.url = url
        self.lock = threading.Lock()
        self.latest = None
        self.error = 'Waiting for first frame'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                with requests.get(self.url, stream=True, timeout=(3, 2)) as response:
                    response.raise_for_status()
                    buffer = b''
                    for chunk in response.iter_content(chunk_size=4096):
                        if self.stop.is_set():
                            return
                        buffer += chunk
                        if len(buffer) > 4_000_000:
                            raise ValueError('No valid JPEG within 4 MB; check endpoint')
                        while True:
                            start = buffer.find(b'\xff\xd8')
                            end = buffer.find(b'\xff\xd9', max(0, start + 2))
                            if start < 0 or end < 0:
                                break
                            encoded = np.frombuffer(buffer[start:end + 2], dtype=np.uint8)
                            buffer = buffer[end + 2:]
                            frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
                            if frame is not None:
                                with self.lock:
                                    self.latest = (time.monotonic(), frame)
                                    self.error = ''
                with self.lock:
                    if not self.error:
                        self.error = 'Feed ended without error (camera rebooted?)'
                self.stop.wait(0.1)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                self.stop.wait(1.0)


class ArmPOC(Node):
    def __init__(self):
        super().__init__('arm_poc')
        defaults = {
            'camera_url': '', 'dry_run': True, 'serial_port': '',
            'home_pose': [90, 90, 90, 90], 'trigger_pose': [90, 90, 90, 90],
            'min_angles': [80, 80, 80, 80], 'max_angles': [100, 100, 100, 100],
            'target_color': 'red', 'color_fraction': 0.05, 'stable_frames': 3,
            'hold_seconds': 3.0, 'camera_stale_seconds': 3.0,
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        self.p['calibration'] = self.declare_parameter(
            'calibration', [],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)).value or []
        if not self.p['camera_url'].startswith(('http://', 'https://')):
            raise ValueError('Set camera_url to the actual HTTP JPEG/MJPEG endpoint')
        if not 0 < self.p['color_fraction'] <= 1:
            raise ValueError('color_fraction must be in (0, 1]')
        self.ranges = color_ranges(self.p['target_color'])
        self.model = self.build_calibration()
        if not all(np.isfinite(self.p[k]) and self.p[k] > 0 for k in
                   ('hold_seconds', 'camera_stale_seconds')):
            raise ValueError('Timeouts must be finite and positive')
        self.home_pose = [int(a) for a in self.p['home_pose']]
        self.target_pose = [int(a) for a in self.p['trigger_pose']]
        self.home = move_command(self.home_pose, self.p['min_angles'], self.p['max_angles'])
        self.trigger_cmd = move_command(self.target_pose, self.p['min_angles'], self.p['max_angles'])
        self.gate = DetectionGate(self.p['stable_frames'])
        self.port = None
        self.camera = None
        self.fault = False
        self.command = self.home
        self.pose = self.home_pose
        self.phase = 'idle'
        self.deadline = 0.0
        self.last_frame = 0.0
        self.started = time.monotonic()
        self.last_sent = 0.0
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.detected = self.create_publisher(Bool, '/vision/color_detected', 10)
        self.joints = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.target = self.create_publisher(PointStamped, '/vision/target_point', 10)
        if not self.p['dry_run']:
            self.port = serial.Serial(self.p['serial_port'], 115200, timeout=0.3, write_timeout=0.3)
            try:
                time.sleep(2)  # USB opening may reset ESP32.
                self.port.reset_input_buffer()
                self.exchange(b'PING\n', b'READY')
            except Exception:
                self.port.close()
                raise
        self.camera = CameraReader(self.p['camera_url'])
        self.started = time.monotonic()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            ('Dry run enabled' if self.p['dry_run'] else 'Hardware mode: commanding home')
            + f", tracking {self.p['target_color']}")

    def build_calibration(self):
        """Parse the flat calibration array into an image-to-joint model.

        Empty means uncalibrated, in which case the node keeps the original
        behaviour of commanding one fixed trigger pose.
        """
        flat = list(self.p['calibration'])
        if not flat:
            self.get_logger().info('No calibration: using the fixed trigger pose')
            return None
        if len(flat) % 6:
            raise ValueError('calibration needs six numbers per point: u, v and four angles')
        samples = [(flat[i], flat[i + 1], flat[i + 2:i + 6]) for i in range(0, len(flat), 6)]
        for _, _, angles in samples:
            move_command(angles, self.p['min_angles'], self.p['max_angles'])
        model = fit_pixel_to_joints(samples)
        self.get_logger().info(f'Calibrated from {len(samples)} points: reaching for the object')
        return model

    def locate(self, mask):
        """Largest matching blob as (fraction of frame, u, v).

        The largest connected blob is used rather than every matching pixel, so
        scattered noise of the right hue cannot masquerade as an object and the
        centroid belongs to one thing rather than the average of several.
        """
        count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
        if count < 2:
            return 0.0, 0.0, 0.0
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        area = float(stats[index, cv2.CC_STAT_AREA])
        cx, cy = centroids[index]
        height, width = mask.shape[:2]
        return area / mask.size, cx / width, cy / height

    def exchange(self, command, expected):
        self.port.write(command)
        reply = self.port.readline().strip()
        if reply != expected:
            raise RuntimeError(f'Controller reply {reply!r}; expected {expected!r}')

    def halt(self, reason):
        self.fault = True
        self.get_logger().error(reason + '; restart node after resolving')
        if self.port:
            try:
                self.port.write(b'STOP\n')
            except serial.SerialException:
                pass

    def tick(self):
        now = time.monotonic()
        with self.camera.lock:
            latest, error = self.camera.latest, self.camera.error
        frame_time = latest[0] if latest else self.started
        if now - frame_time > self.p['camera_stale_seconds']:
            if not self.fault:
                self.halt('Camera feed stale: ' + (error or 'no frames received'))
            return
        if self.fault:
            return
        if latest and latest[0] != self.last_frame:
            self.last_frame, frame = latest
            message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'camera_optical_frame'
            self.images.publish(message)
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = None
            for low, high in self.ranges:
                band = cv2.inRange(hsv, low, high)
                mask = band if mask is None else (mask | band)
            fraction, u, v = self.locate(mask)
            seen = bool(fraction >= self.p['color_fraction'])
            self.detected.publish(Bool(data=seen))
            if seen:
                point = PointStamped()
                point.header.stamp = message.header.stamp
                point.header.frame_id = 'camera_optical_frame'
                point.point.x, point.point.y, point.point.z = u, v, fraction
                self.target.publish(point)
            event = self.gate.update(seen)
            if event and self.phase == 'idle':
                if self.model:
                    self.pose = pose_from_pixel(self.model, u, v,
                                                self.p['min_angles'], self.p['max_angles'])
                    self.command = move_command(self.pose, self.p['min_angles'],
                                                self.p['max_angles'])
                    self.get_logger().info(
                        f'Object at ({u:.2f}, {v:.2f}); reaching {self.pose}')
                else:
                    self.command = self.trigger_cmd
                    self.pose = self.target_pose
                self.phase = 'target'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
        if self.phase != 'idle' and now >= self.deadline:
            if self.phase == 'target':
                self.command = self.home
                self.pose = self.home_pose
                self.phase = 'returning'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Returning home')
            else:
                self.phase = 'idle'
        state = JointState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.name = ['joint0', 'joint1', 'joint2', 'joint3']
        state.position = [math.radians(a) for a in self.pose]
        self.joints.publish(state)
        if now - self.last_sent >= 0.5:
            try:
                if self.port:
                    self.exchange(self.command, b'OK')
                self.last_sent = now
            except Exception as exc:
                self.halt(str(exc))

    def close(self):
        if self.camera:
            self.camera.stop.set()
            self.camera.thread.join(timeout=0.5)
        if self.port:
            try:
                self.port.write(b'STOP\n')
            except serial.SerialException:
                pass
            finally:
                self.port.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmPOC()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
