FROM ros:galactic-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
        ros-$ROS_DISTRO-rviz2 \
        # ros-$ROS_DISTRO-rviz-common \
        # ros-$ROS_DISTRO-rviz-default-plugins \
        # ros-$ROS_DISTRO-rviz-visual-tools \
        ros-$ROS_DISTRO-nav2-rviz-plugins \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp && \
    apt-get upgrade -y && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

CMD ["ros2", "run", "rviz2", "rviz2"]