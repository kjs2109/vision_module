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
        # 'pixel_format': 'BayerRG8',
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
        'chunk_enable_timestamp': True},
    'chameleon': {
        'debug': False,
        'compute_brightness': False,
        'dump_node_map': False,
        # set parameters defined in chameleon.yaml
        'gain_auto': 'Continuous',
        'exposure_auto': 'Continuous',
        'offset_x': 0,
        'offset_y': 0,
        'image_width': 2048,
        'image_height': 1536,
        'pixel_format': 'RGB8',  # 'BayerRG8, 'RGB8' or 'Mono8'
        'frame_rate_continous': True,
        'frame_rate': 100.0,
        'trigger_mode': 'Off',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True},
    'grasshopper': {
        'debug': False,
        'compute_brightness': False,
        'dump_node_map': False,
        # set parameters defined in grasshopper.yaml
        'gain_auto': 'Continuous',
        'exposure_auto': 'Continuous',
        'frame_rate_auto': 'Off',
        'frame_rate': 100.0,
        'trigger_mode': 'Off',
        'chunk_mode_active': True,
        'chunk_selector_frame_id': 'FrameID',
        'chunk_enable_frame_id': True,
        'chunk_selector_exposure_time': 'ExposureTime',
        'chunk_enable_exposure_time': True,
        'chunk_selector_gain': 'Gain',
        'chunk_enable_gain': True,
        'chunk_selector_timestamp': 'Timestamp',
        'chunk_enable_timestamp': True}
    }


def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""
    # parameter_file = LaunchConfig('parameter_file').perform(context)
    # camera_type = LaunchConfig('camera_type').perform(context)
    serial_number = "22495523"
    camera_type = 'blackfly_s'
    parameter_file = PathJoinSubstitution(
        [FindPackageShare('pixkit_sensor_kit_launch'), 'config', camera_type + '.yaml'])
    
    camerainfo_url = 'file://${ROS_HOME}/../pix/parameter/sensor_kit/pixkit_sensor_kit_description/intrinsic_parameters/flir_top_12mm.yaml'
    
    node = Node(package='spinnaker_camera_driver',
                executable='camera_driver_node',
                output='screen',
                name="top_gmsl",
                namespace="top/gmsl",
                parameters=[example_parameters[camera_type],
                            {'parameter_file': parameter_file,
                             'frame_id': 'camera_top_link',
                             'serial_number': serial_number,
                             'camerainfo_url': camerainfo_url
                             }
                            ],
                )

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription([
        LaunchArg('camera_name', default_value=['flir_camera'], description='camera name (ros node name)'),
        LaunchArg('camera_type', default_value='blackfly_s', description='type of camera (blackfly_s, chameleon...)'),
        LaunchArg('serial', default_value="'20435008'", description='FLIR serial number of camera (in quotes!!)'),
        LaunchArg('parameter_file', default_value='22485236', description='path to ros parameter definition file (override camera type)'),
        OpaqueFunction(function=launch_setup)
    ])
