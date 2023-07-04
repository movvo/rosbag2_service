#!/bin/bash 

docker run -it --rm  \
    --volume=$(dirname $(dirname $(realpath $0))):/ros2_ws/src:rw \
    --env=ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    --env=RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \
    --net=host \
    rosbag2_service_node:latest