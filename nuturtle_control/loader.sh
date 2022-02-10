#!/usr/bin/env bash
export ROS_MASTER_URI=http://banana:11311
source /home/msr/install/setup.bash
exec "$@"