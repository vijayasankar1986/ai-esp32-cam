"""LAN status dashboard. Observes ROS; never sends servo commands."""
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
from urllib.parse import urlsplit

ROOT = Path(__file__).resolve().parents[1]
ASSETS = Path(__file__).resolve().parent
RUNTIME = Path(tempfile.mkdtemp(prefix='arm-dashboard-'))
LOCK = threading.Lock()
# Signals waiting MJPEG clients that a new frame landed, so the stream is
# driven by arrivals rather than polling.
FRAME_READY = threading.Condition()
STATE = {'ros': False, 'ros_error': '', 'image_count': 0, 'last_image': 0,
         'last_detection': 0, 'red': None, 'publishers': 0, 'jpeg': None,
         'joints': None, 'last_joints': 0,
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

        def on_detection(msg):
            with LOCK:
                STATE['red'] = msg.data
                STATE['last_detection'] = time.monotonic()

        def on_joints(msg):
            with LOCK:
                STATE['joints'] = [round(math.degrees(a), 1) for a in msg.position]
                STATE['last_joints'] = time.monotonic()

        node.create_subscription(Image, '/camera/image_raw', on_image, qos_profile_sensor_data)
        node.create_subscription(JointState, '/arm/joint_states', on_joints, 10)
        node.create_subscription(Bool, '/vision/color_detected', on_detection, 10)

        def graph():
            with LOCK:
                STATE['publishers'] = node.count_publishers('/camera/image_raw')

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
    data['ros_distro'] = os.environ.get('ROS_DISTRO', 'not sourced')
    data['ros_domain'] = os.environ.get('ROS_DOMAIN_ID', '0')
    data['load'] = round(os.getloadavg()[0], 2)
    try:
        data['test']['telemetry'] = json.loads((RUNTIME / 'test.json').read_text())
    except (OSError, ValueError):
        data['test']['telemetry'] = None
    return data


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

    def log_message(self, *args):
        pass


if __name__ == '__main__':
    threading.Thread(target=ros_observer, daemon=True).start()
    print('Arm dashboard listening on port 8080', flush=True)
    ThreadingHTTPServer(('0.0.0.0', 8080), Handler).serve_forever()
