#!/bin/bash

docker run -it --rm \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    rosbag2_service:latest \
    ros2 launch rosbag2_service start_launch.py