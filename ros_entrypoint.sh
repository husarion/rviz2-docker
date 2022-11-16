#!/bin/bash
set -e

if [[ -v FASTRTPS_DEFAULT_PROFILES_FILE ]]; then
    auxfile="/dds-config-aux.xml"
    cp --attributes-only --preserve $FASTRTPS_DEFAULT_PROFILES_FILE $auxfile
    cat $FASTRTPS_DEFAULT_PROFILES_FILE | envsubst > $auxfile
    export FASTRTPS_DEFAULT_PROFILES_FILE=$auxfile
fi

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros2_ws/install/setup.bash"

exec "$@"
