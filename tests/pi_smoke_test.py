"""Run on the Pi after sourcing ROS and the built workspace; never opens serial."""
import json
import os
from pathlib import Path
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool

from arm_poc.node import ArmPOC

state = {'red': True, 'offline': False}


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if state['offline']:
            self.send_error(503)
            return
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        frame[:, :] = (0, 0, 255) if state['red'] else (0, 255, 0)
        ok, data = cv2.imencode('.jpg', frame)
        assert ok
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data.tobytes())

    def log_message(self, *args):
        pass


server = ThreadingHTTPServer(('127.0.0.1', 0), Handler)
worker = threading.Thread(target=server.serve_forever, daemon=True)
worker.start()
rclpy.init(args=['--ros-args', '-p', f'camera_url:=http://127.0.0.1:{server.server_port}/image',
                 '-p', 'dry_run:=true', '-p', 'hold_seconds:=0.4',
                 '-p', 'camera_stale_seconds:=1.5'])
node = None
try:
    node = ArmPOC()
    images, detections, joints, phases = [], [], [], set()
    image_sub = node.create_subscription(Image, '/camera/image_raw', images.append, qos_profile_sensor_data)
    detection_sub = node.create_subscription(Bool, '/vision/red_detected', lambda msg: detections.append(msg.data), 10)
    joint_sub = node.create_subscription(JointState, '/arm/joint_states', joints.append, 10)

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
            phases.add(node.phase)
            if os.environ.get('ARM_POC_REPORT'):
                report = Path(os.environ['ARM_POC_REPORT'])
                pending = report.with_suffix('.tmp')
                pending.write_text(json.dumps({'frames': len(images), 'phase': node.phase,
                                              'red': detections[-1] if detections else None,
                                              'joints': list(node.pose),
                                              'fault': node.fault}))
                pending.replace(report)
            if images and os.environ.get('ARM_POC_PREVIEW'):
                image = node.bridge.imgmsg_to_cv2(images[-1], desired_encoding='bgr8')
                ok, jpeg = cv2.imencode('.jpg', image)
                if ok:
                    preview = Path(os.environ['ARM_POC_PREVIEW'])
                    pending = preview.with_suffix('.tmp')
                    pending.write_bytes(jpeg.tobytes())
                    pending.replace(preview)

    spin_for(2.5)
    assert node.port is None, 'Dry run unexpectedly opened serial'
    assert images and images[-1].encoding == 'bgr8', 'Image publication failed'
    assert True in detections and 'target' in phases and 'returning' in phases
    assert joints and len(joints[-1].position) == 4, 'Joint states not published'
    state['red'] = False
    spin_for(1.0)
    assert detections[-1] is False and node.gate.armed, 'Removal did not rearm'
    state['offline'] = True
    spin_for(2.0)
    assert node.fault, 'Lost camera did not latch a fault'
    print(f'PASS: {len(images)} images; {len(joints)} joint states; red detection, preset phases, removal rearm, stale-feed fault; serial unopened')
finally:
    if node:
        node.close()
        node.destroy_node()
    rclpy.shutdown()
    server.shutdown()
    server.server_close()
