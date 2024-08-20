#!/bin/env bash 

source config.sh

# Down the bridges
# sudo ip link set br-sim down
for i in $(seq 0 $((n_car-1))); do
    sudo ip link set br-$i down
done


# Remove the taps from the bridges
# sudo ip link set tap-sim nomaster
for i in $(seq 0 $((n_car-1))); do
    sudo ip link set tap-$i nomaster
done

# Delete the bridges
# sudo ip link del br-sim
for i in $(seq 0 $((n_car-1))); do
    sudo ip link del br-$i
done

# Delete the taps
# sudo ip link del tap-sim
for i in $(seq 0 $((n_car-1))); do
    sudo ip link del tap-$i
done

# sudo ip link del internal-sim
for i in $(seq 0 $((n_car-1))); do
    sudo ip link del internal-$i
done

# sudo ip link del external-sim
for i in $(seq 0 $((n_car-1))); do
    sudo ip link del external-$i
done

sudo rm -rf /var/run/netns/*

ls /var/run/netns/

# Verify if still any container existed
docker ps -a
