#!/usr/bin/env bash
# Fetch the SSD MobileNet v3 COCO model for arm_vision.
#
# The weights are 48 MB compressed and the generated text graph is 180 KB, so
# none of it belongs in git. This puts everything in ~/models, which is where
# the detector looks by default.
#
# OpenCV cannot read TensorFlow's frozen graph alone; it needs a text graph
# describing the same network. That file is not in the tarball and is not
# hosted anywhere dependable, so it is generated here with OpenCV's own
# tf_text_graph_ssd.py. The script parses the protobuf itself, so this needs
# no TensorFlow install.
set -euo pipefail

DEST="${1:-$HOME/models}"
BASE="ssd_mobilenet_v3_large_coco_2020_01_14"
mkdir -p "$DEST"
cd "$DEST"

if [ ! -f "$BASE/frozen_inference_graph.pb" ]; then
  echo "Downloading weights (48 MB)..."
  curl -fL --progress-bar -o "$BASE.tar.gz" \
    "http://download.tensorflow.org/models/object_detection/$BASE.tar.gz"
  tar xzf "$BASE.tar.gz"
  rm -f "$BASE.tar.gz"
fi

if [ ! -f ssd_mobilenet_v3.pbtxt ]; then
  echo "Generating the OpenCV text graph..."
  for f in tf_text_graph_ssd.py tf_text_graph_common.py; do
    [ -f "$f" ] || curl -fsSL -o "$f" \
      "https://raw.githubusercontent.com/opencv/opencv/master/samples/dnn/$f"
  done
  python3 tf_text_graph_ssd.py \
    --input "$BASE/frozen_inference_graph.pb" \
    --config "$BASE/pipeline.config" \
    --output ssd_mobilenet_v3.pbtxt
fi

echo "Model ready in $DEST:"
ls -1 ssd_mobilenet_v3.pbtxt "$BASE/frozen_inference_graph.pb"
