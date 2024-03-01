ARG ROS_DISTRO=humble
ARG PREFIX=

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-base AS robot-models-builder
SHELL ["/bin/bash", "-c"]

ARG ROS_DISTRO
ARG PREFIX

WORKDIR /ros2_ws/src

# Clone packages with descriptions
# ROSbot 2
RUN git clone https://github.com/husarion/rosbot_ros.git && \
    find rosbot_ros -mindepth 1 -maxdepth 1 ! -name 'rosbot_description' -exec rm -r {} + && \
    # ROSbot XL
    git clone https://github.com/husarion/rosbot_xl_ros.git && \
    find rosbot_xl_ros -mindepth 1 -maxdepth 1 ! -name 'rosbot_xl_description' -exec rm -r {} + && \
    # Panther
    git clone -b ros2-devel https://github.com/husarion/panther_ros.git && \
    find panther_ros -mindepth 1 -maxdepth 1 ! -name 'panther_description' -exec rm -r {} + && \
    # ros components
    git clone https://github.com/husarion/ros_components_description.git && \
    # OpenManipulatorX
    git clone https://github.com/husarion/open_manipulator_x.git && \
    find open_manipulator_x -mindepth 1 -maxdepth 1 ! -name 'open_manipulator_x_description' -exec rm -r {} + && \
    # DepthAI
    git clone https://github.com/luxonis/depthai-ros.git && \
    find depthai-ros -mindepth 1 -maxdepth 1 ! -name 'depthai_descriptions' -exec rm -r {} +

 # ffmpeg image transport plugin
RUN apt update && apt install -y \
        ros-$ROS_DISTRO-cv-bridge && \
    git clone https://github.com/ros-misc-utilities/ffmpeg_image_transport.git && \
    vcs import . < ./ffmpeg_image_transport/ffmpeg_image_transport.repos

WORKDIR /ros2_ws

# Build packages
RUN rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --from-paths src --ignore-src -y && \
    MYDISTRO=${PREFIX:-ros}; MYDISTRO=${MYDISTRO//-/} && \
    source /opt/$MYDISTRO/$ROS_DISTRO/setup.bash && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

FROM husarnet/ros:${PREFIX}${ROS_DISTRO}-ros-core
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
        ros-$ROS_DISTRO-teleop-twist-keyboard \
        ros-$ROS_DISTRO-rviz2 \
        ros-$ROS_DISTRO-rviz-common \
        ros-$ROS_DISTRO-rviz-default-plugins \
        ros-$ROS_DISTRO-rviz-visual-tools \
        ros-$ROS_DISTRO-rviz-rendering \
        ros-$ROS_DISTRO-nav2-rviz-plugins \
        # for ffmpeg image transport
        ros-$ROS_DISTRO-cv-bridge \
        # allows compressed and theora encoded streams to be received over image_transport
        ros-$ROS_DISTRO-image-transport-plugins && \
    apt-get upgrade -y && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

COPY --from=robot-models-builder /ros2_ws/install /ros2_ws/install

RUN echo $(dpkg -s ros-$ROS_DISTRO-rviz2 | grep 'Version' | sed -r 's/Version:\s([0-9]+.[0-9]+.[0-9]+).*/\1/g') >> /version.txt

CMD ["ros2", "run", "rviz2", "rviz2"]
