#!/bin/bash

FOLDER=$(dirname $0)/rosbag2_service_node
CONFIG_FILE=$FOLDER/config/params.yaml

# Configuration file
if test -f "$CONFIG_FILE"; then
    echo "$CONFIG_FILE already exists"
else
    echo "$CONFIG_FILE doesn't exist, creating"
    cp ${FOLDER}/config/params.sample.yaml ${CONFIG_FILE}
fi

if [ $# -eq 0 ]; then
    echo "Build of rosbag2_service. Building as latest"
    docker build -t rosbag2_service:latest .
else
    echo "Build of rosbag2_service. Building as $1"
    docker build -t rosbag2_service:$1 --build-arg TAG=$1 .
fi