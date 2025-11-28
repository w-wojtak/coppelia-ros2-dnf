#!/bin/bash

# Source ROS2 Humble
source /opt/ros/humble/setup.bash

# Source internal build workspace if it exists
if [ -f /home/rosuser/ros2_ws/install/setup.bash ]; then
    source /home/rosuser/ros2_ws/install/setup.bash
fi

exec "$@"
