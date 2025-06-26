ros2 bag record -o ../bags/camera_lidar_gnss \
                /sensing/lidar/top/pointcloud_raw \
                /sensing/camera/camera0/camera_info \
                /sensing/camera/camera0/image_compressed \
                /sensing/camera/camera_driver/image_raw \
                /sensing/gnss/chc/heading \
                /sensing/gnss/chc/imu \
                /sensing/gnss/chc/pitch \
                /sensing/gnss/fix \
                /sensing/gnss/gnss_fixed \
                /sensing/gnss/gnss_pose \
                /sensing/gnss/gnss_pose_cov \
                /sensing/gnss/heading \
                /sensing/gnss/navpvt \
                /sensing/gnss/time_reference \
                /sensing/gnss/vel \
                /sensing/imu/imu_data \
                /sensing/vehicle_velocity_converter/twist_with_covariance \
                /vehicle/status/velocity_status \
                /tf_static \
                /tf