ARG TAG=latest
FROM geo_interfaces:${TAG} as geo_inter
FROM ros:humble

ARG ROS_DISTRO=humble
ARG WORKSPACE=/ros2_ws

WORKDIR $WORKSPACE

# source overlay from entrypoint
ENV WORKSPACE $WORKSPACE

RUN mkdir /log/rosbag

RUN sed --in-place \
      's|^source .*|source "$WORKSPACE/install/setup.bash"|' \
      /ros_entrypoint.sh

# Install dependencies
RUN apt update -qq && \
    apt upgrade -qq -y --with-new-pkgs && \
    apt install -qq -y \
      valgrind \
      ros-$ROS_DISTRO-rmw-fastrtps-cpp \
      ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
      ros-$ROS_DISTRO-test-msgs \
    && rm -rf /var/lib/apt/lists/*

COPY ./rosbag2_service_msg ./src/rosbag2_service_msg
COPY ./rosbag2_service_node ./src/rosbag2_service_node
COPY --from=geo_inter /ros2_ws/src/geo_interfaces /ros2_ws/src/geo_interfaces


RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build \
      --symlink-install