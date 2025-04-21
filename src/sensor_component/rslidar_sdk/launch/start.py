from launch.actions import OpaqueFunction
from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration('namespace').perform(context) 
    return [
        Node(
            namespace=namespace,
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            remappings=[(f'{namespace}/rs/points', f'{namespace}/top/pointcloud_raw')],
            #remappings=[(f'{namespace}/rs/points', f'{namespace}/concatenated/pointcloud')],
            output='screen'
        )
    ]

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='/sensing/lidar',
        description='Namespace for the rslidar_sdk node'
    )

    return LaunchDescription([
        namespace_arg,
        OpaqueFunction(function=launch_setup) 
    ])