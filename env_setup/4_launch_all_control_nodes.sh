#!/bin/bash

source config.sh

for i in $(seq 0 $((n_car-1))); do
    gnome-terminal --window-with-profile=default --title="car_control_$i" -e "bash -c \"
        docker exec -it car_control_$i bash -c '
            source install/setup.bash
            ros2 launch f1tenth_gym_ros launch_car$i.py'
        ; exec bash\"" &
done