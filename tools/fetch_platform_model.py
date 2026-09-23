#!/usr/bin/env python3
"""Download the promoted vision-platform model for the arm's detector.

Label frames, train and promote in the vision-platform console, then run this
on the Pi. It fetches the project's active model (model.onnx + labels.json)
into ~/models/platform/, where arm_vision's detector loads it:

    VP_PASSWORD=... tools/fetch_platform_model.py --email you@example.com --project arm-objects
    ros2 launch arm_vision detector.launch.py model:=~/models/platform/model.onnx

The password comes from VP_PASSWORD so it stays out of shell history.
"""
import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path


def call(url, token=None, body=None):
    headers = {'Content-Type': 'application/json'}
    if token:
        headers['Authorization'] = f'Bearer {token}'
    data = json.dumps(body).encode() if body is not None else None
    request = urllib.request.Request(url, data=data, headers=headers)
    with urllib.request.urlopen(request, timeout=15) as response:
        return json.load(response)


def open_project(api, email, project):
    """Log in and resolve a project slug or id. Exits with a readable reason."""
    password = os.environ.get('VP_PASSWORD')
    if not password:
        sys.exit('Set VP_PASSWORD to the vision-platform password.')
    try:
        token = call(f'{api}/auth/login', body={'email': email, 'password': password})['access_token']
        projects = call(f'{api}/projects', token)
    except urllib.error.HTTPError as exc:
        sys.exit(f'API error {exc.code}: {exc.read().decode(errors="replace")}')
    except urllib.error.URLError as exc:
        sys.exit(f'Cannot reach {api}: {exc.reason}. Is docker compose up on the PC?')
    match = [p for p in projects if project in (p['slug'], p['id'])]
    if not match:
        sys.exit(f"No project '{project}'. Have: {', '.join(p['slug'] for p in projects)}")
    return token, match[0]


def download(url, path):
    tmp = path.with_suffix(path.suffix + '.part')
    with urllib.request.urlopen(url, timeout=60) as response:
        tmp.write_bytes(response.read())
    tmp.replace(path)          # Never leave a half-written model in place.


def main():
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--api', default='http://192.168.1.7:8000',
                        help='vision-platform API (the PC running docker compose)')
    parser.add_argument('--email', required=True)
    parser.add_argument('--project', required=True, help='project slug or id')
    parser.add_argument('--out', default='~/models/platform')
    args = parser.parse_args()

    api = args.api.rstrip('/')
    token, project = open_project(api, args.email, args.project)
    config = call(f"{api}/projects/{project['id']}/config", token)

    model = config['active_model']
    if not model:
        sys.exit('No promoted model yet: train, then Promote a version on the Models page.')
    if not model.get('labels_url'):
        sys.exit('The promoted model is not a YOLO model (was it trained with TRAINER=yolo?).')

    out = Path(os.path.expanduser(args.out))
    out.mkdir(parents=True, exist_ok=True)
    try:
        download(model['labels_url'], out / 'labels.json')
        download(model['download_url'], out / 'model.onnx')
    except urllib.error.URLError as exc:
        sys.exit(f'Download failed ({exc}). S3_PUBLIC_ENDPOINT_URL in vision-platform/.env '
                 'must be an address the Pi can reach, not localhost.')

    labels = json.loads((out / 'labels.json').read_text())
    (out / 'version.json').write_text(json.dumps(
        {'project': project['slug'], 'version': model['version'], 'map50': model['eval_score']}))
    print(f"v{model['version']} (mAP50 {model['eval_score']:.2f}) -> {out}/model.onnx")
    print('Classes: ' + ', '.join(labels['names']))


if __name__ == '__main__':
    main()
