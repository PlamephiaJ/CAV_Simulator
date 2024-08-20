#!/bin/env bash

echo "Configuring network!"

set -e

source config.sh

# Add bridges
echo "Add bridges..."
for i in $(seq 0 $((n_car-1))); do
    sudo ip link add name br-$i type bridge
done
echo "Done."

# Add tap devices
echo "Add tap devices..."
for i in $(seq 0 $((n_car-1))); do
    sudo ip tuntap add tap-$i mode tap
done

for i in $(seq 0 $((n_car-1))); do
    sudo ifconfig tap-$i 0.0.0.0 promisc up
done
echo "Done."

# Attach tap devices to bridges and activate
echo "Attach taps to bridges..."

for i in $(seq 0 $((n_car-1))); do
    sudo ip link set tap-$i master br-$i
    sudo ip link set br-$i up
done
echo "Done."

# disallow bridge traffic to go through ip tables chain
# See: https://unix.stackexchange.com/questions/499756/how-does-iptable-work-with-linux-bridge
# and: https://wiki.libvirt.org/page/Net.bridge.bridge-nf-call_and_sysctl.conf
pushd /proc/sys/net/bridge
    for f in bridge-nf-*; do echo 0 > $f; done
popd

# Create the network namespace runtime folder if not exists
sudo mkdir -p /var/run/netns

export pid_arr=()

for i in $(seq 0 $((n_car-1))); do
    container_name=$(printf "car_control_%d" $i)
    pid=$(docker inspect --format '{{ .State.Pid }}' $container_name)
    pid_arr+=("$pid")
done

# Soft-link the network namespace created by container into the linux namespace runtime
for i in $(seq 0 $((n_car-1))); do
    pid=${pid_arr[$i]}
    sudo ln -s /proc/$pid/ns/net /var/run/netns/$pid
done

# Create Veth pair to attach to bridge
echo "Create Veth pairs..."

for i in $(seq 0 $((n_car-1))); do
    sudo ip link add internal-$i type veth peer name external-$i
    sudo ip link set internal-$i master br-$i
    sudo ip link set internal-$i up
    echo "$i veth pair internal side complete"
done

sim_ip_address=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' sim_env)
echo "Container sim eth0 IP address: $sim_ip_address"

for i in $(seq 0 $((n_car-1))); do
    echo "$i accept IP address: $sim_ip_address"
    sudo ip netns exec ${pid_arr[$i]} iptables -A INPUT -i eth0 -s $sim_ip_address -j ACCEPT
    sudo ip netns exec ${pid_arr[$i]} iptables -A INPUT -i eth0 -j DROP

    sudo ip link set external-$i netns ${pid_arr[$i]}
    sudo ip netns exec ${pid_arr[$i]} ip link set dev external-$i name eth1

    mac_prefix="12:34:56:78:9A"
    mac_suffix=$(printf "%02X" $((0x01 + i)))
    mac_address="${mac_prefix}:${mac_suffix}"
    sudo ip netns exec ${pid_arr[$i]} ip link set eth1 address $mac_address
    
    sudo ip netns exec ${pid_arr[$i]} ip link set eth1 up
    sudo ip netns exec ${pid_arr[$i]} ip addr add 172.20.0.$((2 + i))/16 dev eth1

    echo "$i container side configuration complete"
done

echo "Done."
echo "### Setup complete. Ready to start simulation ###"