##############
# Base image #
##############
FROM osrf/ros:noetic-desktop-full as base
# FROM ros:noetic-ros-base as base
# FROM ros:noetic-robot as base

ARG CATKIN_WORKSPACE_DIR=${CATKIN_WORKSPACE_DIR}

SHELL ["/bin/bash", "-c"]

RUN apt update \
  && apt install -y \
  git \
  python3-catkin-tools \
  python3-osrf-pycommon \  
#  ros-noetic-pcl-ros \
#  ros-noetic-pcl-conversions \
#  ros-noetic-pcl-msgs \
  ros-noetic-tf2-sensor-msgs \
  ros-noetic-tf2

COPY ../src ${CATKIN_WORKSPACE_DIR}/src

WORKDIR ${CATKIN_WORKSPACE_DIR}

RUN source /opt/ros/noetic/setup.bash \
 && rosdep install -i --from-path src --rosdistro noetic -y \
 && catkin build \
 && echo 'source ${CATKIN_WORKSPACE_DIR}/devel/setup.bash' >> /ros_entrypoint.sh

RUN sed -i '/exec "$@"/d' /ros_entrypoint.sh \
 && echo 'exec "$@"' >> /ros_entrypoint.sh

ENTRYPOINT [ "/ros_entrypoint.sh" ]

