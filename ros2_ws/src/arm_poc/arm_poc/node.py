import math
import socket
import threading
import time

import cv2
import numpy as np
import requests
import serial
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from .logic import (DetectionGate, clean_reply, color_ranges, fit_pixel_to_joints,
                    move_command, pose_from_pixel)


class CameraReader:
    """Bounded JPEG extraction for snapshot and MJPEG HTTP responses."""

    def __init__(self, url, connect_timeout=5.0, read_timeout=10.0):
        self.url = url
        self.timeout = (connect_timeout, read_timeout)
        # Keep-alive matters more than anything else here. Measured against an
        # AI-Thinker serving snapshots: 0.7 fps opening a connection per frame
        # against 4.4 fps reusing one. The camera was never the bottleneck.
        self.session = requests.Session()
        self.lock = threading.Lock()
        self.latest = None
        self.error = 'Waiting for first frame'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while not self.stop.is_set():
            try:
                with self.session.get(self.url, stream=True,
                                      timeout=self.timeout) as response:
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


class DeviceCameraReader:
    """USB webcam on the Pi, read through V4L2. Same interface as CameraReader.

    Selected when camera_url is a device path such as /dev/video0. Prefer the
    /dev/v4l/by-id/ link: video0 and video1 can swap between boots, and the
    by-id name follows the camera. MJPG is requested because most webcams only
    reach full frame rate in it; raw YUYV over USB 2 is far slower.
    """

    def __init__(self, device, width=640, height=480):
        self.device = device
        self.size = (width, height)
        self.lock = threading.Lock()
        self.latest = None
        self.error = 'Waiting for first frame'
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def open(self):
        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f'Cannot open {self.device} (unplugged, or bbt '
                               'not in the video group?)')
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.size[0])
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.size[1])
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # Newest frame, not a queue.
        return capture

    def run(self):
        while not self.stop.is_set():
            capture = None
            try:
                capture = self.open()
                while not self.stop.is_set():
                    ok, frame = capture.read()
                    if not ok or frame is None:
                        raise RuntimeError(f'{self.device} stopped delivering frames')
                    with self.lock:
                        self.latest = (time.monotonic(), frame)
                        self.error = ''
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                self.stop.wait(1.0)
            finally:
                if capture is not None:
                    capture.release()


class TcpPort:
    """Wraps a TCP socket so exchange()/drain()/handshake() below can talk to
    the arm controller's Wi-Fi command port exactly as they talk to a
    pyserial Serial object: write(bytes), readline() and read(n) that return
    b'' on timeout rather than raising, reset_input_buffer(), and close().
    """

    def __init__(self, host, port, timeout):
        self.timeout = timeout
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self._buf = bytearray()

    def write(self, data):
        self.sock.sendall(data)

    def _fill(self, want):
        try:
            chunk = self.sock.recv(max(want, 256))
        except (socket.timeout, OSError):
            return False
        if not chunk:
            return False
        self._buf.extend(chunk)
        return True

    def read(self, size=1):
        if not self._buf and not self._fill(size):
            return b''
        out = bytes(self._buf[:size])
        del self._buf[:size]
        return out

    def readline(self):
        while b'\n' not in self._buf:
            if not self._fill(256):
                return b''
        idx = self._buf.index(b'\n') + 1
        line = bytes(self._buf[:idx])
        del self._buf[:idx]
        return line

    def reset_input_buffer(self):
        # No boot-ROM chatter to flush on Wi-Fi the way opening a USB serial
        # port has; this only drops whatever the controller sent before we
        # started reading, e.g. a reply to a command from a prior connection.
        self._buf.clear()
        self.sock.settimeout(0)
        try:
            while self.sock.recv(4096):
                pass
        except OSError:
            pass
        finally:
            self.sock.settimeout(self.timeout)

    def close(self):
        try:
            self.sock.close()
        except OSError:
            pass


class ArmPOC(Node):
    def __init__(self):
        super().__init__('arm_poc')
        defaults = {
            'camera_url': '', 'dry_run': True, 'serial_port': '',
            'control_host': '', 'control_port': 3333,
            'home_pose': [90, 90, 90, 90], 'trigger_pose': [90, 90, 90, 90],
            'min_angles': [80, 80, 80, 80], 'max_angles': [100, 100, 100, 100],
            'target_color': 'red', 'color_fraction': 0.05, 'stable_frames': 3,
            'hold_seconds': 3.0, 'camera_stale_seconds': 8.0,
            'connect_timeout': 5.0, 'read_timeout': 10.0, 'auto_recover': True,
            'allow_manual': False, 'manual_timeout': 2.0, 'serial_timeout': 1.0,
            'gripper_joint': 3, 'gripper_open': 120, 'gripper_closed': 60,
            'grasp_seconds': 1.5, 'pick_enabled': False,
        }
        self.p = {k: self.declare_parameter(k, v).value for k, v in defaults.items()}
        # dynamic_typing because an empty default would otherwise be inferred as
        # a byte array and clash with the double array actually supplied.
        self.p['calibration'] = self.declare_parameter(
            'calibration', [], ParameterDescriptor(dynamic_typing=True)).value or []
        if not self.p['camera_url'].startswith(('http://', 'https://', '/dev/')):
            raise ValueError('Set camera_url to an HTTP JPEG/MJPEG endpoint '
                             'or a /dev/ video device')
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
        self.fault_kind = None
        self.command = self.home
        self.pose = self.home_pose
        self.phase = 'idle'
        self.pick_reach = None
        self.deadline = 0.0
        self.last_frame = 0.0
        self.started = time.monotonic()
        self.last_sent = 0.0
        self.bridge = CvBridge()
        self.images = self.create_publisher(Image, '/camera/image_raw', qos_profile_sensor_data)
        self.detected = self.create_publisher(Bool, '/vision/color_detected', 10)
        self.joints = self.create_publisher(JointState, '/arm/joint_states', 10)
        self.target = self.create_publisher(PointStamped, '/vision/target_point', 10)
        if self.p['allow_manual']:
            self.create_subscription(JointState, '/arm/manual_pose', self.on_manual, 10)
            self.get_logger().warn(
                'Manual control ENABLED: /arm/manual_pose can move the arm')
        if not self.p['dry_run']:
            if self.p['control_host']:
                self.port = TcpPort(self.p['control_host'], int(self.p['control_port']),
                                    self.p['serial_timeout'])
            elif self.p['serial_port']:
                self.port = serial.Serial(self.p['serial_port'], 115200,
                                          timeout=self.p['serial_timeout'],
                                          write_timeout=self.p['serial_timeout'])
            else:
                raise ValueError(
                    'Set control_host (Wi-Fi) or serial_port (USB) when dry_run is false')
            try:
                if isinstance(self.port, serial.Serial):
                    time.sleep(2)  # USB opening resets the ESP32.
                self.handshake()
            except Exception:
                self.port.close()
                raise
        if self.p['camera_url'].startswith('/dev/'):
            self.camera = DeviceCameraReader(self.p['camera_url'])
        else:
            self.camera = CameraReader(self.p['camera_url'], self.p['connect_timeout'],
                                       self.p['read_timeout'])
        # Serial runs on its own thread. A blocked write must never stall the
        # timer callback: a 1 s timeout plus a retry was starving image
        # publishing whenever the controller struggled, so a power problem on
        # the arm blanked the camera too.
        self.writer_stop = threading.Event()
        self.writer = None
        if self.port:
            self.writer = threading.Thread(target=self.write_loop, daemon=True)
            self.writer.start()
        self.started = time.monotonic()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            ('Dry run enabled' if self.p['dry_run'] else 'Hardware mode: commanding home')
            + f", tracking {self.p['target_color']}")

    def on_manual(self, message):
        """Apply a jogged pose, if manual control is enabled and safe to accept.

        Manual commands expire after manual_timeout, so a dashboard that closes
        its tab or loses the network cannot leave the arm holding a pose
        indefinitely. Angles go through move_command, so the same limit checks
        that guard the vision path guard this one.
        """
        if self.fault:
            return
        angles = [int(round(math.degrees(a))) for a in message.position]
        if len(angles) != 4:
            self.get_logger().warn('Manual pose ignored: need exactly four joints')
            return
        try:
            command = move_command(angles, self.p['min_angles'], self.p['max_angles'])
        except ValueError as exc:
            self.get_logger().warn(f'Manual pose rejected: {exc}')
            return
        self.command = command
        self.pose = angles
        self.phase = 'manual'
        self.deadline = time.monotonic() + self.p['manual_timeout']
        self.last_sent = 0.0

    def apply(self, pose):
        """Set the commanded pose, validated against the joint limits."""
        self.command = move_command(pose, self.p['min_angles'], self.p['max_angles'])
        self.pose = list(pose)

    def with_gripper(self, pose, angle):
        """Copy a pose with the gripper joint overridden, clamped to limits."""
        out = list(pose)
        j = int(self.p['gripper_joint'])
        lo, hi = int(self.p['min_angles'][j]), int(self.p['max_angles'][j])
        out[j] = max(lo, min(hi, int(angle)))
        return out

    def begin_pick(self, u, v, now):
        """Start a pick: approach with the gripper open, then grasp and lift.

        The reach pose comes from the calibrated image-to-joint map, so the arm
        goes where the object actually is. Only the gripper joint is driven by
        the sequence.
        """
        reach = pose_from_pixel(self.model, u, v,
                                self.p['min_angles'], self.p['max_angles'])
        self.pick_reach = reach
        self.apply(self.with_gripper(reach, self.p['gripper_open']))
        self.phase = 'approach'
        self.deadline = now + self.p['hold_seconds']
        self.get_logger().info(
            f'Object at ({u:.2f}, {v:.2f}); approaching {self.pose} with gripper open')

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

    def write_loop(self):
        """Send the current command to the controller, off the callback thread."""
        while not self.writer_stop.is_set():
            if self.fault:
                self.writer_stop.wait(0.5)
                continue
            command = self.command
            try:
                self.exchange(command, b'OK')
            except Exception as exc:
                self.halt(str(exc), kind='controller')
            self.writer_stop.wait(0.4)

    def drain(self, quiet=0.3, limit=4.0):
        """Read until the controller has gone quiet.

        Opening the port resets the ESP32, whose boot ROM chatters at 74880
        baud. At 115200 that arrives as junk, the firmware rejects the long
        line with 'ERR line too long', and that queued reply then sits one
        ahead of every later command, so PING reads the junk reply and MOVE
        reads READY. A fixed sleep is not enough because junk keeps arriving
        after the flush; draining to silence keeps replies aligned.
        """
        deadline = time.monotonic() + limit
        last_data = time.monotonic()
        while time.monotonic() < deadline:
            if self.port.read(256):
                last_data = time.monotonic()
            elif time.monotonic() - last_data >= quiet:
                return

    def handshake(self, attempts=3):
        for attempt in range(attempts):
            self.drain()
            self.port.reset_input_buffer()
            self.port.write(b'PING\n')
            reply = clean_reply(self.port.readline())
            if reply == b'READY':
                self.drain(quiet=0.2, limit=1.0)  # Drop any duplicate READY.
                return
            self.get_logger().warn(
                f'Handshake attempt {attempt + 1}: got {reply!r}, expected READY')
        raise RuntimeError('Controller never answered PING with READY')

    def exchange(self, command, expected, retries=1):
        """Send a command and require its reply, tolerating one stale line.

        A reply left over from an earlier command sits one ahead of everything
        after it, so a MOVE reads READY and looks like a hard failure when the
        controller is actually fine. Draining and retrying once recovers from
        that; a genuinely dead controller still fails on the retry.
        """
        for attempt in range(retries + 1):
            self.port.write(command)
            reply = clean_reply(self.port.readline())
            if reply == expected:
                return
            self.drain(quiet=0.2, limit=1.0)   # Resync before deciding.
            if attempt < retries:
                self.get_logger().warn(
                    f'Stale reply {reply!r} for {command.strip()!r}; resyncing')
                continue
            hint = (' (no reply: controller may have reset, check servo power)'
                    if reply == b'' else '')
            raise RuntimeError(
                f'Controller reply {reply!r}; expected {expected!r}{hint}')

    def halt(self, reason, kind='camera'):
        self.fault = True
        self.fault_kind = kind
        recoverable = kind == 'camera' and self.p['auto_recover']
        suffix = ('; will resume automatically when frames return' if recoverable
                  else '; restart the node after resolving')
        self.get_logger().error(reason + suffix)
        if self.port:
            try:
                self.port.write(b'STOP\n')
            except (serial.SerialException, OSError):
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
        if self.fault and self.fault_kind == 'camera':
            # Only camera faults clear themselves. A controller that stopped
            # answering will not start again because frames resumed, and
            # retrying it every tick just floods the log.
            if not self.p['auto_recover']:
                return
            # Frames are flowing again. Resume from home with a fresh gate, so a
            # recovery can never continue a motion that was interrupted midway.
            self.fault = False
            self.command = self.home
            self.pose = self.home_pose
            self.phase = 'idle'
            self.gate = DetectionGate(self.p['stable_frames'])
            self.last_sent = 0.0
            self.get_logger().info('Camera feed restored; resuming from home')
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
            if event and self.phase == 'idle' and not self.fault:
                if self.model and self.p['pick_enabled']:
                    self.begin_pick(u, v, now)
                elif self.model:
                    self.apply(pose_from_pixel(self.model, u, v,
                                               self.p['min_angles'],
                                               self.p['max_angles']))
                    self.get_logger().info(
                        f'Object at ({u:.2f}, {v:.2f}); reaching {self.pose}')
                    self.phase = 'target'
                    self.deadline = now + self.p['hold_seconds']
                else:
                    self.command = self.trigger_cmd
                    self.pose = self.target_pose
                    self.phase = 'target'
                    self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
        if self.phase != 'idle' and now >= self.deadline and not self.fault:
            if self.phase == 'manual':
                # The jog stopped refreshing. Go home rather than hold a pose
                # nobody is watching any more.
                self.command = self.home
                self.pose = self.home_pose
                self.phase = 'returning'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Manual control released; returning home')
            elif self.phase == 'approach':
                # In position with the gripper open: close it on the object.
                self.apply(self.with_gripper(self.pick_reach, self.p['gripper_closed']))
                self.phase = 'grasp'
                self.deadline = now + self.p['grasp_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Closing gripper')
            elif self.phase == 'grasp':
                # Lift by going home, keeping the gripper closed so the object
                # comes with it.
                self.apply(self.with_gripper(self.home_pose, self.p['gripper_closed']))
                self.phase = 'lift'
                self.deadline = now + self.p['hold_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Lifting to home with the object')
            elif self.phase == 'lift':
                self.apply(self.with_gripper(self.home_pose, self.p['gripper_open']))
                self.phase = 'returning'
                self.deadline = now + self.p['grasp_seconds']
                self.last_sent = 0.0
                self.get_logger().info('Releasing object')
            elif self.phase == 'target':
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
        # Commanding happens on the writer thread; nothing here blocks on
        # serial, so images and joint states keep flowing regardless.

    def close(self):
        self.writer_stop.set()
        if self.writer:
            self.writer.join(timeout=1.5)
        if self.camera:
            self.camera.stop.set()
            self.camera.thread.join(timeout=0.5)
        if self.port:
            try:
                self.port.write(b'STOP\n')
            except (serial.SerialException, OSError):
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
