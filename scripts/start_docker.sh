#!/bin/bash

docker_name="slim_bridge"

docker run --detach --privileged --net=host \
--name $docker_name \
-v $PWD/initial_topics.yaml:/root/bridge_ws/install/slim_bridge/share/slim_bridge/config/initial_topics.yaml \
 slim_bridge:latest ./cmd_script.sh