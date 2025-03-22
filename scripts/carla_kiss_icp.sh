#!/bin/bash

echo "Starting CARLA server..."
gnome-terminal -- bash -c "cd ~/carla-0915/CARLA_0.9.15 && ./CarlaUE4.sh; exec bash"

sleep 10  

# echo "Starting Sensing Visualization..."
# gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch autoware_carla_interface carla_sensing_visualizer.launch.xml; exec bash"

echo "carla-autowrare interface" 
gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch autoware_carla_interface autoware_carla_interface.launch.xml carla_map:=Town03; exec bash"


echo "Starting Ego Vehicle..."
gnome-terminal -- bash -c "source ~/carla-0915/carla-0915-env/bin/activate && python3 ./ego_manual_control.py; exec bash"

sleep 10

echo "launch kiss-icp" 
gnome-terminal -- bash -c "source ../install/setup.bash && ros2 launch kiss_icp odometry.launch.py topic:=/sensing/lidar/concatenated/pointcloud invert_odom_tf:=True; exec bash"
