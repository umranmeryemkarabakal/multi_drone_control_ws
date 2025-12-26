#!/bin/bash

# Drone 1
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 0 --out=udp:127.0.0.1:14551 --map --console; exec bash"

# Drone 2
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 1 --out=udp:127.0.0.1:14552 --map --console; exec bash"

# Drone 3
gnome-terminal -- bash -c "cd ~/ardupilot/ArduCopter && sim_vehicle.py -v ArduCopter -f gazebo-iris --instance 2 --out=udp:127.0.0.1:14553 --map --console; exec bash"
