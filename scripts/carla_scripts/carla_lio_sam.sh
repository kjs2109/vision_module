#!/bin/bash

echo "Starting CARLA server..."
gnome-terminal -- bash -c "cd ~/carla-0915/CARLA_0.9.15 && ./CarlaUE4.sh; exec bash"

sleep 10  

# echo "Starting Sensing Visualization..."
# gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch autoware_carla_interface carla_sensing_visualizer.launch.xml; exec bash"

echo "carla-autowrare interface" 
gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml; exec bash"


echo "Starting Ego Vehicle..."
gnome-terminal -- bash -c "source ~/carla-0915/carla-0915-env/bin/activate && python3 ./ego_manual_control.py; exec bash"

sleep 10

echo "launch lio_sam" 
gnome-terminal -- bash -c "ource ../install/setup.bash && ros2 launch lio_sam run.launch.py params_file:=/media/k/part11/vision_module/src/autoware/external/lio_sam/config/params_carla.yaml"