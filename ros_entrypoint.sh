#!/bin/bash
set -e

if [[ -v FASTRTPS_DEFAULT_PROFILES_FILE ]]; then
    envsubst < $FASTRTPS_DEFAULT_PROFILES_FILE > $FASTRTPS_DEFAULT_PROFILES_FILE
fi

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"
