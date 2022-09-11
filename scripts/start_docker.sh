#!/bin/bash

if [[ -z "${ROBOT_NAME}" ]]; then
  docker_name=slim_bridge
else
  docker_name=${ROBOT_NAME}_slim_bridge
fi

docker run --detach --privileged --net=host \
--name $docker_name \
--env RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
--env ROBOT_NAME=$ROBOT_NAME \
--env CYCLONEDDS_URI=file:///ddsconfig.xml \
-v $PWD/ddsconfig.xml:/ddsconfig.xml \
-v $PWD/initial_topics.yaml:/root/bridge_ws/install/slim_bridge/share/slim_bridge/config/initial_topics.yaml \
 slim_bridge:latest ./cmd_script.sh