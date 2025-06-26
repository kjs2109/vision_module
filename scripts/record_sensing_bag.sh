#!/bin/bash

ros2 bag record -o ../bags/test1 \
                /sensing/lidar/top/pointcloud_raw \
                /sensing/camera/camera0/camera_info \
                /sensing/camera/camera0/image_compressed \
                /sensing/camera/camera0/image_raw \
                /sensing/camera/camera_driver/camera_info \
                /sensing/camera/camera_driver/image_raw/compressed \
                /sensing/camera/camera_driver/image_raw \
                /sensing/vehicle_velocity_converter/twist_with_covariance \
                /tf \
                /tf_static \
                /robot_description \
                # --split --max-bag-duration 300 \
                # --compression-mode file \
                # --compression-format lz4f
