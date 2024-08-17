#!/bin/bash

source config.sh

workspace_dir="/home/plm/multi_sim_ws"

rviz_container_command="rocker --nvidia --x11 --volume ./src:/sim_ws/src --volume ./waypoint:/sim_ws/waypoint --volume ./measure:/sim_ws/measure --name sim_env --hostname sim_env --network sim -- f1tenth_gym_ros"

gnome-terminal --working-directory="$workspace_dir" --window-with-profile=default -e "bash -c '$rviz_container_command; exec bash'"

# 定义要运行的命令模板
control_container_template="docker run -it --rm --name car_control_%d --hostname car_control_%d --network sim --volume ./src:/sim_ws/src --volume ./waypoint:/sim_ws/waypoint --volume ./measure:/sim_ws/measure f1tenth_gym_ros"

# 启动 n_car 个新的shell窗口,并在每个窗口运行上面定义的命令
for i in $(seq 0 $((n_car-1))); do
    command=$(printf "$control_container_template" $i $i)
    gnome-terminal --working-directory="$workspace_dir" --window-with-profile=default -e "bash -c '$command; exec bash'"
done