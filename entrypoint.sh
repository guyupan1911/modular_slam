#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 2
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 2 ${ROS_DISTRO}"

# Source the base workspace, if built
if [ -f /modular_slam_ws/install/setup.bash ]
then
  source /modular_slam_ws/install/setup.bash
  echo "Sourced modular_slam workspace"
fi

# Execute the command passed into this entrypoint
exec "$@"
