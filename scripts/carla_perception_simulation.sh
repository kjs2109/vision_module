#!/bin/bash

echo "Starting CARLA server..."
gnome-terminal -- bash -c "cd ~/carla-0915/CARLA_0.9.15 && ./CarlaUE4.sh; exec bash"

sleep 10  

echo "Starting Sensing Visualization..."
gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch autoware_carla_interface carla_sensing_visualizer.launch.xml; exec bash"

echo "Starting Ego Vehicle..."
gnome-terminal -- bash -c "source ~/carla-0915/carla-0915-env/bin/activate && python3 ./ego_manual_control.py; exec bash"

