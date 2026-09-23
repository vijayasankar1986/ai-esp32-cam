#!/usr/bin/env python3
"""Capture training frames from the arm's camera into vision-platform.

Run on the Pi while the dashboard is up. It takes a frame from the dashboard
every --every seconds (the node owns the webcam, so the device itself is not
free to open), then uploads them to the project as one dataset, ready for
labelling in the console's Labeling page:

    VP_PASSWORD=... tools/capture_to_platform.py --email you@example.com --project arm-objects

Move the objects around between shots: varied positions, angles and lighting
teach the model far more than many copies of one scene. 40-100 frames with a
few boxes each is a sensible first dataset.

Frames are packed into a video at 1 fps because the platform extracts one
frame per second of video, so every captured frame becomes exactly one
labelable frame.
"""
import argparse
import os
import tempfile
import time
import urllib.request

import cv2
import numpy as np

from fetch_platform_model import call, open_project


def grab(dashboard):
    with urllib.request.urlopen(f'{dashboard}/api/frame', timeout=5) as response:
        frame = cv2.imdecode(np.frombuffer(response.read(), np.uint8), cv2.IMREAD_COLOR)
    if frame is None:
        raise RuntimeError('Dashboard returned no decodable frame')
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--api', default='http://192.168.1.7:8000')
    parser.add_argument('--dashboard', default='http://localhost:8080')
    parser.add_argument('--email', required=True)
    parser.add_argument('--project', required=True, help='project slug or id')
    parser.add_argument('--frames', type=int, default=40)
    parser.add_argument('--every', type=float, default=2.0, help='seconds between frames')
    args = parser.parse_args()

    api = args.api.rstrip('/')
    token, project = open_project(api, args.email, args.project)

    frames = []
    print(f'Capturing {args.frames} frames, one every {args.every}s. Move the objects between shots.')
    while len(frames) < args.frames:
        try:
            frames.append(grab(args.dashboard))
            print(f'  {len(frames)}/{args.frames}', flush=True)
        except Exception as exc:
            print(f'  skipped: {exc}', flush=True)
        time.sleep(args.every)

    height, width = frames[0].shape[:2]
    name = time.strftime('arm-%Y%m%d-%H%M%S.mp4')
    path = os.path.join(tempfile.mkdtemp(), name)
    writer = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*'mp4v'), 1.0, (width, height))
    for frame in frames:
        writer.write(cv2.resize(frame, (width, height)))
    writer.release()

    base = f"{api}/projects/{project['id']}/datasets"
    upload = call(f'{base}/upload-url', token, {'filename': name, 'content_type': 'video/mp4'})
    with open(path, 'rb') as handle:
        request = urllib.request.Request(upload['upload_url'], data=handle.read(),
                                         method='PUT', headers={'Content-Type': 'video/mp4'})
    urllib.request.urlopen(request, timeout=60).close()
    call(f"{base}/{upload['dataset_id']}/complete", token, {})
    os.remove(path)
    print(f'Uploaded {len(frames)} frames as {name}. Label them on the Labeling page.')


if __name__ == '__main__':
    main()
