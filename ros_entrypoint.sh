#!/bin/bash
set -e
# Загружаем рабочие области ROS 2
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/photobot_ws/install/setup.bash"  --
exec "$@"