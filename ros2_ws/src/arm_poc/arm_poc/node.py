import threading
import time

import cv2
import numpy as np
import requests
import serial
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from .logic import DetectionGate, move_command


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
            'red_fraction': 0.05, 'stable_frames': 3,
            'hold_seconds': 3.0, 'camera_stale_seconds': 3.0,
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        if not self.p['camera_url'].startswith(('http://', 'https://')):
            raise ValueError('Set camera_url to the actual HTTP JPEG/MJPEG endpoint')
        if not 0 < self.p['red_fraction'] <= 1:
            raise ValueError('red_fraction must be in (0, 1]')
        if not all(np.isfinite(self.p[k]) and self.p[k] > 0 for k in
                   ('hold_seconds', 'camera_stale_seconds')):
            raise ValueError('Timeouts must be finite and positive')
        self.home = move_command(self.p['home_pose'], self.p['min_angles'], self.p['max_angles'])
        self.target = move_command(self.p['trigger_pose'], self.p['min_angles'], self.p['max_angles'])
        self.gate = DetectionGate(self.p['stable_frames'])
        self.port = None
        self.camera = None
        self.fault = False
        self.command = self.home
        self.phase = 'idle'
        self.deadline = 0.0
        self.last_frame = 0.0
        self.started = time.monotonic()
        self.last_sent = 0.0
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.detected = self.create_publisher(Bool, '/vision/red_detected', 10)
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
        self.get_logger().info('Dry run enabled' if self.p['dry_run'] else 'Hardware mode: commanding home')

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
                self.halt('Camera feed stale: ' + error)
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
            mask = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
            mask |= cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
            red = bool(np.count_nonzero(mask) / mask.size >= self.p['red_fraction'])
            self.detected.publish(Bool(data=red))
            event = self.gate.update(red)
            if event and self.phase == 'idle':
                self.command = self.target
                self.phase = 'target'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Red detected: trigger pose')
        if self.phase != 'idle' and now >= self.deadline:
            if self.phase == 'target':
                self.command = self.home
                self.phase = 'returning'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Returning home')
            else:
                self.phase = 'idle'
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
