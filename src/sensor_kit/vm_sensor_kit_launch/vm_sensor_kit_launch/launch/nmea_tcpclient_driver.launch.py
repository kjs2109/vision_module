# Copyright 2022 Open Source Robotics Foundation, Inc.
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

""" A simple launch file for the nmea_tcpclient_driver node. """

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchIntrospector, LaunchService, substitutions
from launch_ros import actions
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    """Generate a launch description for a single tcpclient driver."""
    logger = substitutions.LaunchConfiguration("log_level")
    driver_node = actions.Node(
        package='nmea_navsat_driver',
        executable='nmea_tcpclient_driver',
        output='screen',
        parameters=[{
            "ip": "192.168.1.102",
            "port": 9904,
            "buffer_size": 4096
        }],
        arguments=['--ros-args', '--log-level', logger]
        )
    argument = DeclareLaunchArgument(
            "log_level",
            default_value=["error"],
            description="Logging level"
            )
    return LaunchDescription([argument, driver_node])


def main(argv):
    ld = generate_launch_description()

    print('Starting introspection of launch description...')
    print('')

    print(LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    ls = LaunchService()
    ls.include_launch_description(ld)
    return ls.run()


if __name__ == '__main__':
    main(sys.argv)
