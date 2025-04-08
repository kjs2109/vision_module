# Copyright 2023 Pixmoving, Inc. All rights reserved.
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

import os
from ament_index_python import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):
    
    cropbox_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::CropBoxFilterComponent",
        name="crop_box_filter",
        remappings=[
            ("input", "/sensing/lidar/concatenated/pointcloud"),
            ("output", "/localization/util/measurement_range/pointcloud"),  # /localization/util/
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "min_x": -1.0,
                "max_x": 1.0,
                "min_y": -0.5,
                "max_y": 0.5,
                "min_z": -0.5,
                "max_z": 1.8,
                "negative": True,
            }
        ],
    )

    voxel_grid_downsample_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::VoxelGridDownsampleFilterComponent",
        name="voxel_grid_downsample_filter",
        remappings=[
            ("input", "/localization/util/measurement_range/pointcloud"),
            ("output", "/localization/util/voxel_grid_downsample/pointcloud"),  # /localization/util/
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "voxel_size_x": 0.3,
                "voxel_size_y": 0.3,
                "voxel_size_z": 0.1,
            }
        ],
    )

    random_downsample_component = ComposableNode(
        package="pointcloud_preprocessor",
        plugin="autoware::pointcloud_preprocessor::RandomDownsampleFilterComponent",
        name="random_downsample_filter",
        remappings=[
            ("input", "/localization/util/voxel_grid_downsample/pointcloud"),
            ("output", "/localization/util/downsample/pointcloud"),  # /localization/util/
        ],
        parameters=[
            {
                "input_frame": LaunchConfiguration("base_frame"),
                "output_frame": LaunchConfiguration("base_frame"),
                "sample_num": 1500,
            }
        ],
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name=LaunchConfiguration("container_name"),
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[cropbox_component, voxel_grid_downsample_component, random_downsample_component],
        condition=IfCondition(LaunchConfiguration("use_pointcloud_container")),
        output="screen",
    )

    return [container,]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("base_frame", "base_link")
    add_launch_arg("use_intra_process", "False")
    add_launch_arg("use_multithread", "False")
    add_launch_arg("use_concat_filter", "True")
    add_launch_arg("use_pointcloud_container", "True")
    add_launch_arg("container_name", "pointcloud_preprocessor_container")

    set_container_executable = SetLaunchConfiguration(
        "container_executable", "component_container", condition=UnlessCondition(LaunchConfiguration("use_multithread")),
    )

    set_container_mt_executable = SetLaunchConfiguration(
        "container_executable", "component_container_mt", condition=IfCondition(LaunchConfiguration("use_multithread")),
    )

    return launch.LaunchDescription(
        launch_arguments
        + [set_container_executable, set_container_mt_executable]
        + [OpaqueFunction(function=launch_setup)]
    )