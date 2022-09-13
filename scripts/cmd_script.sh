#!/bin/bash

source /root/bridge_ws/install/setup.bash

ros1_running=$(ps -e | grep roscore)

if [ -z $ros1_running ]
then
      echo "ROS1 not running, starting..."
      roscore &

else
      echo "ROS1 already running"
fi
ros2 launch slim_bridge bridge.launch.py
