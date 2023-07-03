ARG TAG=latest
FROM ros:humble

ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws

WORKDIR $WORKSPACE

# source overlay from entrypoint
ENV WORKSPACE $WORKSPACE

RUN sed --in-place \
      's|^source .*|source "$WORKSPACE/install/setup.bash"|' \
      /ros_entrypoint.sh

# Install dependencies
RUN apt-get update -qq && \
    apt-get upgrade -qq -y --with-new-pkgs && \
    apt-get install -qq -y \
      valgrind \
      ros-$ROS_DISTRO-rmw-fastrtps-cpp \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    apt-get remove -y ros-humble-rosbag2-cpp \
    && rm -rf /var/lib/apt/lists/*

COPY ./rosbag2_service_msg ./src/rosbag2_service_msg
COPY ./rosbag2_service_node ./src/rosbag2_service_node

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install