#!/bin/bash

if [[ -z "${ROBOT_NAME}" ]]; then
  docker_name=slim_bridge
else
  docker_name=${ROBOT_NAME}_slim_bridge
fi
docker stop $docker_name && docker rm $docker_name