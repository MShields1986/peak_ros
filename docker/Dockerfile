##############
# Base image #
##############
FROM osrf/ros:jazzy-desktop-full AS base
# FROM ros:jazzy-ros-base AS base

ARG ROS2_WORKSPACE_DIR=/ros2_ws

SHELL ["/bin/bash", "-c"]

RUN apt update \
  && apt install -y \
  git \
  build-essential \
  python3-colcon-common-extensions \
  ros-jazzy-pcl-ros \
  ros-jazzy-tf2-sensor-msgs \
  ros-jazzy-tf2 \
  && rm -rf /var/lib/apt/lists/*

COPY src ${ROS2_WORKSPACE_DIR}/src

WORKDIR ${ROS2_WORKSPACE_DIR}

RUN source /opt/ros/jazzy/setup.bash \
 && rosdep update \
 && rosdep install -i --from-path src --rosdistro jazzy -y \
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
RUN source /opt/ros/jazzy/setup.bash \
 && colcon build --symlink-install --cmake-args -DBUILD_TESTING=ON
