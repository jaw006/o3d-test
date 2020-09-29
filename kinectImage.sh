#!/bin/bash
# https://docs.microsoft.com/en-us/azure/kinect-dk/record-file-format
FILENAME=$1
TEMP_DIR="/c/temp/"
RECORDER="/c/Program Files/Azure Kinect SDK v1.2.0/tools/k4arecorder.exe"
FFMPEG="/d/ffmpeg/bin/"
OPS="--device 0 -d NFOV_UNBINNED -c 720p -l 0.1 -e 0 ${TEMP_DIR}${FILENAME}.mkv"

echo ${FILENAME}

if [ "${FILENAME}" == "" ]; then
	echo "usage: ./kinectImage.sh <FILENAME>"
	exit
fi

# Record video
"${RECORDER}" ${OPS}

# Extract frames
