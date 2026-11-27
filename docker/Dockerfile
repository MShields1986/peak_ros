##############
# Base image #
##############
FROM osrf/ros:lyrical-desktop-full AS base
# FROM ros:lyrical-ros-base AS base

ARG ROS2_WORKSPACE_DIR=/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN apt update \
  && apt install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  ros-lyrical-pcl-ros \
  ros-lyrical-tf2-sensor-msgs \
  ros-lyrical-tf2 \
  && rm -rf /var/lib/apt/lists/*

COPY src ${ROS2_WORKSPACE_DIR}/src

WORKDIR ${ROS2_WORKSPACE_DIR}

RUN source /opt/ros/lyrical/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro lyrical -y \
 && colcon build --symlink-install \
 && echo "source ${ROS2_WORKSPACE_DIR}/install/setup.bash" >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]
