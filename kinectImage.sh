#!/bin/bash
FILENAME=$1
KINECT_DIR="/c/Program Files/Azure Kinect SDK v1.2.0/tools"
RECORDER="k4arecorder.exe"
OPS="--device 0 -d NFOV_UNBINNED -c 720p -l 0.1 -e 0 /c/temp/${FILENAME}.mkv"

# Record video
"${KINECT_DIR}/${RECORDER}" ${OPS}
