# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch import LaunchDescription

example_parameters = {
    'blackfly_s': {
        'debug': False,
        'compute_brightness': False,
        'adjust_timestamp': True,
        'dump_node_map': False,
        # set parameters defined in blackfly_s.yaml
        'gain_auto': 'Continuous',
        'pixel_format': 'BGR8',
        'exposure_auto': 'Continuous',
        # These are useful for GigE cameras
        # 'device_link_throughput_limit': 380000000,
        # 'gev_scps_packet_size': 9000,
        # ---- to reduce the sensor width and shift the crop
        # 'image_width': 1408,
        # 'image_height': 1080,
        # 'offset_x': 16,
        # 'offset_y': 0,
        'frame_rate_auto': 'Off',
        'frame_rate': 30.0,
        'frame_rate_enable': True,
        'buffer_queue_size': 1,
        'trigger_mode': 'Off',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': False,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True}
    }


def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""

    serial_number = "22485236"
    camera_type = 'blackfly_s'
    parameter_file = PathJoinSubstitution([FindPackageShare('vm_sensor_kit_launch'), 'config', camera_type + '.yaml'])
    camerainfo_url = 'package://vm_sensor_kit_launch/data/BlackFlyS_81.yaml'
    
    node = Node(package='spinnaker_camera_driver',
                executable='camera_driver_node',
                output='screen',
                parameters=[example_parameters[camera_type],
                            {
                             'parameter_file': parameter_file,
                             'serial_number': serial_number,
                             'camerainfo_url': camerainfo_url, 
                             }
                            ],
                )

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
