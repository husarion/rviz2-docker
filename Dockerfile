ARG ROS_DISTRO=galactic

FROM ros:$ROS_DISTRO-ros-base AS robot-models-builder

# select bash as default shell
SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

RUN apt update

# install everything needed
# ROSbot 2
RUN mkdir -p src/rosbot_ros && \
    pushd src/rosbot_ros && \
    git init && \
    git remote add -f origin https://github.com/husarion/rosbot_ros.git && \
    git sparse-checkout init && \
    git sparse-checkout set "rosbot_description" && \
    git pull origin humble && \
    popd && \
    # ROSbot XL
    mkdir -p src/rosbot_xl_ros && \
    pushd src/rosbot_xl_ros && \
    git init && \
    git remote add -f origin https://github.com/husarion/rosbot_xl_ros.git && \
    git sparse-checkout init && \
    git sparse-checkout set "rosbot_xl_description" && \
    git pull origin master && \
    popd && \
    # ros components
    git clone https://github.com/husarion/ros_components_description.git src/ros_components_description -b ros2 && \
    rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --packages-select rosbot_description rosbot_xl_description ros_components_description

FROM husarnet/ros:$ROS_DISTRO-ros-core

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rviz-common \
        ros-$ROS_DISTRO-rviz-default-plugins \
        ros-$ROS_DISTRO-rviz-visual-tools \
        ros-$ROS_DISTRO-rviz-rendering \
        ros-$ROS_DISTRO-nav2-rviz-plugins && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY ./settings /settings

COPY --from=robot-models-builder /ros2_ws /ros2_ws

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc && \
    echo ". /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["ros2", "run", "rviz2", "rviz2"]