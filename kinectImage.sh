#!/bin/bash
# https://docs.microsoft.com/en-us/azure/kinect-dk/record-file-format

FILENAME=$1
TEMP_DIR="/c/temp/"
OUTPUTVIDEO_PATH="${TEMP_DIR}${FILENAME}.mkv"
OUTPUTFRAMES_PATH="${TEMP_DIR}${FILENAME}/"

RECORDER="/c/Program Files/Azure Kinect SDK v1.2.0/tools/k4arecorder.exe"
RECORDER_OPS="--device 0 -d NFOV_UNBINNED -c 720p -l 0.2 -e 0 ${OUTPUTVIDEO_PATH}"

FFMPEG="/d/ffmpeg/bin/ffmpeg.exe"
FFMPEG_OPS="-i ${OUTPUTVIDEO_PATH} -map 0:1 -vsync 0 ${TEMP_DIR}${FILENAME}/depth%04d.png"

if [ "${FILENAME}" == "" ]; then
	echo "usage: ./kinectImage.sh <FILENAME>"
	exit
fi

if [ ! -d "${OUTPUTFRAMES_PATH}" ]; then
	echo "Making output directory: ${OUTPUTFRAMES_PATH}"
	mkdir "${OUTPUTFRAMES_PATH}"
fi

# Record video
"${RECORDER}" ${RECORDER_OPS}

# Extract frames
"${FFMPEG}" ${FFMPEG_OPS}
