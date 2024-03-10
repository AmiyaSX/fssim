#!/bin/bash
set -e
source /root/.bashrc

ROS_DISTRO=kinetic

source "/opt/ros/$ROS_DISTRO/setup.bash"

CONTAINER_INITIALIZED=".CONTAINER_INITIALIZED_PLACEHOLDER"
if [ ! -e $CONTAINER_INITIALIZED ]; then
    echo "-- First container startup --"
    echo "y" | /ws/src/fssim/update_dependencies.sh
    catkin init
    catkin build
    echo "source /ws/devel/setup.bash" >> /root/.bashrc
    touch $CONTAINER_INITIALIZED
else
    echo "-- Not first container startup --"
    source "/ws/devel/setup.bash"
fi

exec "$@"
