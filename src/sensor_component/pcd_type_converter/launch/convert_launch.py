from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pcd_type_converter',
            executable='convert_node',
            name='pcd_type_converter_node',
            output='screen',
            remappings=[
                ('/raw_lidar', '/sensing/lidar/top/pointcloud_raw'),  
                ('/converted_lidar', '/sensing/lidar/concatenated/pointcloud')  
            ]
        )
    ])
