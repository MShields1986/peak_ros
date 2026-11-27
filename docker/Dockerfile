##############
# Base image #
##############
FROM osrf/ros:humble-desktop-full AS base
# FROM ros:humble-ros-base AS base

ARG ROS2_WORKSPACE_DIR=/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN apt update \
  && apt install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  ros-humble-pcl-ros \
  ros-humble-tf2-sensor-msgs \
  ros-humble-tf2 \
  && rm -rf /var/lib/apt/lists/*

COPY src ${ROS2_WORKSPACE_DIR}/src

WORKDIR ${ROS2_WORKSPACE_DIR}

RUN source /opt/ros/humble/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro humble -y \
 && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF \
 && echo "source ${ROS2_WORKSPACE_DIR}/install/setup.bash" >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]


##############
# Test image #
##############
FROM base AS test

# Build test binaries only (tests are run at container start via docker-compose command)
RUN source /opt/ros/humble/setup.bash \
 && colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
