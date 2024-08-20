#!/bin/bash

set -e

echo "Start building, please wait..."

running_containers=$(docker ps -q)

echo "$running_containers" | parallel --no-notice "docker container exec -t {} bash -c 'source /opt/ros/foxy/setup.bash && colcon build'"