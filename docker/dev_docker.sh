#!/bin/bash 

docker run -it --rm  \
    --volume=$(dirname $(dirname $(realpath $0))):/ros2_ws/src/ast_action_manager:rw \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    ast_action_manager:latest