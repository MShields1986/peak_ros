##############
# Base image #
##############
FROM osrf/ros:rolling-desktop-full AS base
# FROM ros:rolling-ros-base AS base

ARG ROS2_WORKSPACE_DIR=${ROS2_WORKSPACE_DIR}

SHELL ["/bin/bash", "-c"]

RUN apt update \
  && apt install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  ros-rolling-pcl-ros \
  ros-rolling-tf2-sensor-msgs \
  ros-rolling-tf2 \
  && rm -rf /var/lib/apt/lists/*

COPY src ${ROS2_WORKSPACE_DIR}/src

WORKDIR ${ROS2_WORKSPACE_DIR}

RUN source /opt/ros/rolling/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro rolling -y \
 && colcon build --symlink-install \
 && echo "source ${ROS2_WORKSPACE_DIR}/install/setup.bash" >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
