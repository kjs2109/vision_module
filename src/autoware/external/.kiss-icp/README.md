# kiss-icp

```
git clone https://github.com/kjs2109/kiss-icp.git

cd kiss-icp

docker build . -t kissicp_ros2_humble:latest

xhost +local:docker 
docker run -it --privileged --net=host --ipc=host \
    --name kiss_icp_ros2 \
    -e "DISPLAY=$DISPLAY" \
    -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -e "XAUTHORITY=$XAUTH" \
    -v .:/root/kiss-icp
    -v /media/k/part1/data/data_odometry_velodyne/dataset:/data \
    kissicp_ros2_humble:latest 

# execute kiss-icp 
ros2 bag play campus -l
ros2 launch kiss_icp odometry.launch.py topic:=/points_raw

# save pcd map 
ros2 service call /save_local_map_service std_srvs/srv/Trigger {}
```