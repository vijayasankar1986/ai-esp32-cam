"""LAN status dashboard.

Observes ROS. It can also jog the arm, but only when ARM_DASHBOARD_CONTROL=1 is
set in the environment AND the node was started with allow_manual:=true. Both
are off by default, because this server has no authentication and binds every
interface: with control on, anyone who can reach port 8080 can move the arm.
Prefer an SSH tunnel over exposing it.

It never opens the serial port. Jogs are published on /arm/manual_pose and the
ROS node remains the only owner of the controller, so joint limits and the
watchdog stay in one place.
"""
import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
try:
    from sensor_msgs.msg import JointState
except ImportError:
    JointState = None        # Status-only mode without ROS on the path.
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).resolve().parent
RUNTIME = Path(tempfile.mkdtemp(prefix='arm-dashboard-'))
CONTROL = os.environ.get('ARM_DASHBOARD_CONTROL') == '1'
JOG = {'publisher': None, 'pose': [90.0, 90.0, 90.0, 90.0]}
# Captured calibration points: each is u, v and the four angles that reach
# that spot. Written to a file so a teaching session survives a restart.
CALIB = {'points': []}
# Gripper travel, captured from the jog position rather than guessed.
GRIPPER = {'open': None, 'closed': None, 'joint': 3}
CALIB_FILE = Path.home() / 'arm_calibration.json'
LOCK = threading.Lock()
# Signals waiting MJPEG clients that a new frame landed, so the stream is
# driven by arrivals rather than polling.
FRAME_READY = threading.Condition()
STATE = {'ros': False, 'ros_error': '', 'image_count': 0, 'last_image': 0,
         'last_detection': 0, 'red': None, 'publishers': 0, 'jpeg': None,
         'joints': None, 'last_joints': 0, 'arm_node': False,
         'target': None, 'last_target': 0,
         'test': {'status': 'not_run', 'output': '', 'finished': None}}


def text_file(path, fallback=''):
    try:
        return Path(path).read_text().strip().strip('\x00')
    except OSError:
        return fallback


def ros_observer():
    try:
        import cv2
        import rclpy
        from cv_bridge import CvBridge
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import PointStamped
        from sensor_msgs.msg import Image, JointState
        from std_msgs.msg import Bool
        rclpy.init()
        node = Node('arm_dashboard_observer')
        bridge = CvBridge()

        def on_image(msg):
            try:
                frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                if frame.shape[1] > 960:
                    frame = cv2.resize(frame, (960, int(frame.shape[0] * 960 / frame.shape[1])))
                ok, encoded = cv2.imencode('.jpg', frame)
                if ok:
                    with LOCK:
                        STATE['jpeg'] = encoded.tobytes()
                        STATE['image_count'] += 1
                        STATE['last_image'] = time.monotonic()
                with FRAME_READY:
                    FRAME_READY.notify_all()
            except Exception as exc:
                with LOCK:
                    STATE['ros_error'] = str(exc)

        def on_target(msg):
            with LOCK:
                STATE['target'] = [round(msg.point.x, 4), round(msg.point.y, 4),
                                   round(msg.point.z, 4)]
                STATE['last_target'] = time.monotonic()

        def on_detection(msg):
            with LOCK:
                STATE['red'] = msg.data
                STATE['last_detection'] = time.monotonic()

        def on_joints(msg):
            with LOCK:
                STATE['joints'] = [round(math.degrees(a), 1) for a in msg.position]
                STATE['last_joints'] = time.monotonic()

        subs = []
        retry = {'at': 0.0}

        def subscribe():
            for old in subs:
                node.destroy_subscription(old)
            subs.clear()
            subs.append(node.create_subscription(
                Image, '/camera/image_raw', on_image, qos_profile_sensor_data))
            subs.append(node.create_subscription(
                JointState, '/arm/joint_states', on_joints, 10))
            subs.append(node.create_subscription(
                Bool, '/vision/color_detected', on_detection, 10))
            subs.append(node.create_subscription(
                PointStamped, '/vision/target_point', on_target, 10))

        subscribe()
        if CONTROL:
            JOG['publisher'] = node.create_publisher(JointState, '/arm/manual_pose', 10)

        def graph():
            """Watch the graph, and resubscribe if a publisher sends nothing.

            Subscriptions created before the node existed do not reliably pick
            up a later publisher, which left the dashboard reporting a
            connected system with no data until someone restarted it. Rebuilding
            them means startup order no longer matters and a node restart
            recovers on its own.
            """
            images = node.count_publishers('/camera/image_raw')
            now = time.monotonic()
            with LOCK:
                STATE['publishers'] = images
                STATE['arm_node'] = node.count_publishers('/arm/joint_states') > 0
                last = STATE['last_image']
            starved = images and (not last or now - last > 12)
            # Rate-limited by its own clock rather than by faking last_image,
            # which would report a live camera when no frame had arrived.
            if starved and now - retry['at'] > 12:
                retry['at'] = now
                node.get_logger().warn('Publisher present but no images; resubscribing')
                subscribe()

        node.create_timer(2.0, graph)
        with LOCK:
            STATE['ros'] = True
        rclpy.spin(node)
    except Exception as exc:
        with LOCK:
            STATE['ros'] = False
            STATE['ros_error'] = str(exc)


def status():
    with LOCK:
        data = {k: v for k, v in STATE.items() if k != 'jpeg'}
        data['test'] = dict(STATE['test'])
    now = time.monotonic()
    data['frame_age'] = round(now - data.pop('last_image'), 1) if data['last_image'] else None
    data.pop('last_image', None)
    target_time = data.pop('last_target')
    if not target_time or now - target_time > 3:
        data['target'] = None
    data['calibration'] = list(CALIB['points'])
    data['gripper'] = dict(GRIPPER)
    joint_time = data.pop('last_joints')
    if not joint_time or now - joint_time > 3:
        data['joints'] = None
    detection_time = data.pop('last_detection')
    if not detection_time or now - detection_time > 3:
        data['red'] = None
    data['camera_live'] = data['frame_age'] is not None and data['frame_age'] < 3
    data['usb'] = [p.name for p in sorted(Path('/dev/serial/by-id').glob('*'))]
    data['host'] = socket.gethostname()
    data['model'] = text_file('/proc/device-tree/model', 'Raspberry Pi')
    data['uptime_seconds'] = int(float(text_file('/proc/uptime', '0').split()[0]))
    temp = text_file('/sys/class/thermal/thermal_zone0/temp')
    data['temperature'] = round(int(temp) / 1000, 1) if temp.isdigit() else None
    disk = shutil.disk_usage(ROOT)
    data['disk_free_gb'] = round(disk.free / 1024**3, 1)
    data['control'] = CONTROL and JOG['publisher'] is not None
    data['limits'] = joint_limits()
    data['jog_pose'] = list(JOG['pose'])
    data['ros_distro'] = os.environ.get('ROS_DISTRO', 'not sourced')
    data['ros_domain'] = os.environ.get('ROS_DOMAIN_ID', '0')
    data['load'] = round(os.getloadavg()[0], 2)
    try:
        data['test']['telemetry'] = json.loads((RUNTIME / 'test.json').read_text())
    except (OSError, ValueError):
        data['test']['telemetry'] = None
    return data


def joint_limits():
    """Joint limits from the params file the node is started with.

    Read rather than hardcoded, so widening the range in one place is enough
    and the jog buttons cannot offer angles the node would reject. Falls back
    to the conservative starting range if the file cannot be parsed.
    """
    lower, upper = [60] * 4, [120] * 4
    try:
        text = (ROOT / 'ros2_ws/src/arm_poc/config/poc.yaml').read_text()
        for key, target in (('min_angles', 'lower'), ('max_angles', 'upper')):
            for line in text.splitlines():
                stripped = line.strip()
                if stripped.startswith(key + ':'):
                    values = [int(v) for v in
                              stripped.split('[', 1)[1].split(']', 1)[0].split(',')]
                    if len(values) == 4:
                        if target == 'lower':
                            lower = values
                        else:
                            upper = values
                    break
    except (OSError, ValueError, IndexError):
        pass
    return {'min': lower, 'max': upper}


def run_test():
    env = dict(os.environ, ROS_DOMAIN_ID='87', ROS_LOCALHOST_ONLY='1',
               ARM_POC_REPORT=str(RUNTIME / 'test.json'),
               ARM_POC_PREVIEW=str(RUNTIME / 'test.jpg'), PYTHONUNBUFFERED='1')
    try:
        unit = subprocess.run([sys.executable, '-m', 'unittest', 'discover', '-s', 'tests', '-v'],
                              cwd=ROOT, env=env, capture_output=True, text=True, timeout=30)
        smoke = subprocess.run([sys.executable, 'tests/pi_smoke_test.py'], cwd=ROOT,
                               env=env, capture_output=True, text=True, timeout=60)
        passed = unit.returncode == 0 and smoke.returncode == 0
        result = {'status': 'passed' if passed else 'failed', 'finished': time.time(),
                  'output': (unit.stdout + unit.stderr + smoke.stdout + smoke.stderr)[-16000:]}
    except Exception as exc:
        result = {'status': 'failed', 'finished': time.time(), 'output': str(exc)}
    with LOCK:
        STATE['test'] = result


class Handler(BaseHTTPRequestHandler):
    def respond(self, code, body, mime='application/json'):
        if not isinstance(body, bytes):
            body = json.dumps(body).encode()
        self.send_response(code)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('Content-Security-Policy', "default-src 'self'; style-src 'self'; script-src 'self'; img-src 'self'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_GET(self):
        path = urlsplit(self.path).path
        assets = {'/': ('index.html', 'text/html; charset=utf-8'),
                  '/app.js': ('app.js', 'text/javascript'),
                  '/style.css': ('style.css', 'text/css'),
                  '/favicon.svg': ('favicon.svg', 'image/svg+xml')}
        if path in assets:
            name, mime = assets[path]
            return self.respond(200, (ASSETS / name).read_bytes(), mime)
        if path == '/api/status':
            return self.respond(200, status())
        if path == '/api/stream':
            return self.stream_frames()
        if path == '/api/frame':
            with LOCK:
                jpeg = STATE['jpeg']
            return self.respond(200, jpeg, 'image/jpeg') if jpeg else self.respond(404, {'error': 'No camera frame received'})
        if path == '/api/test/frame':
            try:
                return self.respond(200, (RUNTIME / 'test.jpg').read_bytes(), 'image/jpeg')
            except OSError:
                return self.respond(404, {'error': 'No test image yet'})
        self.respond(404, {'error': 'Not found'})

    def stream_frames(self):
        """multipart/x-mixed-replace, the same shape the camera itself serves."""
        boundary = 'armframe'
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=' + boundary)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.end_headers()
        sent = -1
        try:
            while True:
                with FRAME_READY:
                    FRAME_READY.wait(timeout=2.0)
                with LOCK:
                    jpeg, count = STATE['jpeg'], STATE['image_count']
                if jpeg is None or count == sent:
                    continue        # timed out waiting; loop so we notice a dead client
                sent = count
                head = ('--%s\r\nContent-Type: image/jpeg\r\n'
                        'Content-Length: %d\r\n\r\n' % (boundary, len(jpeg)))
                self.wfile.write(head.encode('ascii'))
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass                     # viewer navigated away

    def do_POST(self):
        # No CORS; require a custom same-origin request for the bounded test action.
        origin = self.headers.get('Origin')
        if self.headers.get('X-Arm-Dashboard') != '1' or (origin and urlsplit(origin).netloc != self.headers.get('Host')):
            return self.respond(403, {'error': 'Same-origin request required'})
        if self.path == '/api/jog':
            return self.handle_jog()
        if self.path in ('/api/calibration/capture', '/api/calibration/clear'):
            return self.handle_calibration(self.path.rsplit('/', 1)[1])
        if self.path in ('/api/gripper/open', '/api/gripper/closed'):
            return self.handle_gripper(self.path.rsplit('/', 1)[1])
        if self.path in ('/api/node/restart', '/api/node/stop'):
            return self.handle_node(self.path.rsplit('/', 1)[1])
        if self.path != '/api/test':
            return self.respond(404, {'error': 'Not found'})
        with LOCK:
            if STATE['test']['status'] == 'running':
                return self.respond(409, {'error': 'Test already running'})
            STATE['test'] = {'status': 'running', 'output': '', 'finished': None}
        for name in ('test.json', 'test.jpg'):
            (RUNTIME / name).unlink(missing_ok=True)
        threading.Thread(target=run_test, daemon=True).start()
        self.respond(202, {'status': 'running'})

    def handle_jog(self):
        """Publish one jogged pose. Absolute angles, so a dropped request
        cannot accumulate into a movement nobody asked for."""
        if not CONTROL or JOG['publisher'] is None:
            return self.respond(403, {'error': 'Control disabled. Start with '
                                               'ARM_DASHBOARD_CONTROL=1 and '
                                               'allow_manual:=true'})
        try:
            length = int(self.headers.get('Content-Length', 0))
            body = json.loads(self.rfile.read(length) or b'{}')
            pose = [float(a) for a in body['pose']]
        except (ValueError, KeyError, TypeError):
            return self.respond(400, {'error': 'Body must be {"pose": [a, b, c, d]}'})
        if len(pose) != 4 or not all(0 <= a <= 180 for a in pose):
            return self.respond(400, {'error': 'Need four angles within 0-180'})
        message = JointState()
        message.name = ['joint0', 'joint1', 'joint2', 'joint3']
        message.position = [math.radians(a) for a in pose]
        JOG['publisher'].publish(message)
        with LOCK:
            JOG['pose'] = pose
        return self.respond(200, {'pose': pose})

    def handle_calibration(self, action):
        """Capture or clear a calibration point.

        A point pairs where the object appears in the frame with the joint
        angles that reach it, so it is only meaningful when the object is
        visible and the arm has been jogged to it.
        """
        if not CONTROL:
            return self.respond(403, {'error': 'Control disabled'})
        if action == 'clear':
            CALIB['points'] = []
            self.save_calibration()
            return self.respond(200, {'points': []})
        with LOCK:
            target = STATE['target']
        pose = list(JOG['pose'])
        if not target:
            return self.respond(409, {'error': 'No object detected; nothing to pair with'})
        CALIB['points'].append({'u': target[0], 'v': target[1],
                                'angles': [int(round(a)) for a in pose]})
        self.save_calibration()
        return self.respond(200, {'points': CALIB['points']})

    def handle_gripper(self, which):
        """Record the current gripper angle as the open or closed position.

        Taken from where the operator has actually jogged it, so the value
        reflects this mechanism and this object rather than a guess.
        """
        if not CONTROL:
            return self.respond(403, {'error': 'Control disabled'})
        joint = int(GRIPPER['joint'])
        angle = int(round(JOG['pose'][joint]))
        GRIPPER[which] = angle
        self.save_calibration()
        return self.respond(200, {'open': GRIPPER['open'],
                                  'closed': GRIPPER['closed']})

    def save_calibration(self):
        """Persist points, and emit the YAML line to paste into poc.yaml."""
        flat = []
        for point in CALIB['points']:
            flat += [point['u'], point['v']] + point['angles']
        try:
            CALIB_FILE.write_text(json.dumps(
                {'points': CALIB['points'],
                 'gripper_open': GRIPPER['open'],
                 'gripper_closed': GRIPPER['closed'],
                 'calibration_yaml': 'calibration: [' +
                                     ', '.join(str(v) for v in flat) + ']'}, indent=2))
        except OSError:
            pass

    def handle_node(self, action):
        """Restart or stop the arm_poc node.

        Gated behind the same control flag as jogging, because restarting the
        node reopens the serial port and commands home, which moves the arm.
        The script does the killing and waiting; doing it here would block the
        HTTP thread for seconds.
        """
        if not CONTROL:
            return self.respond(403, {'error': 'Control disabled'})
        script = ASSETS / 'run_node.sh'
        try:
            done = subprocess.run(['/bin/bash', str(script), action],
                                  capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            return self.respond(504, {'error': 'Restart timed out'})
        output = (done.stdout + done.stderr).strip()
        if done.returncode:
            return self.respond(500, {'error': output or 'restart failed'})
        return self.respond(200, {'status': action, 'detail': output})

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    threading.Thread(target=ros_observer, daemon=True).start()
    print('Arm dashboard listening on port 8080', flush=True)
    ThreadingHTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
