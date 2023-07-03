#!/bin/bash

docker run -it --rm \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    ast_action_manager:latest \
    ros2 launch ast_action_manager launch.py