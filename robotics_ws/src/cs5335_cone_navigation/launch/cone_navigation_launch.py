# Copyright 2026 Claire Liu
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

"""
Start the Cone Detector and, optionally, the Cone Navigator with one parameter file.

The navigator asks for target IDs with input(), which needs a terminal, so it runs in its
own xterm window (navigator_prefix). With navigator:=false only the detector starts, to
tune the detection without moving the robot.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the launch description of the cone detector and navigator."""
    default_params = os.path.join(get_package_share_directory('cs5335_cone_navigation'),
                                  'config', 'cone_navigation.yaml')
    params = LaunchConfiguration('params_file')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params,
                              description='Parameter file for both nodes'),
        DeclareLaunchArgument('navigator', default_value='true',
                              description='Also start the Cone Navigator (moves the robot)'),
        DeclareLaunchArgument('navigator_prefix', default_value='xterm -e',
                              description='Command that gives the navigator a terminal'),
        Node(package='cs5335_cone_navigation', executable='cone_detector',
             name='cone_detector', parameters=[params], output='screen'),
        Node(package='cs5335_cone_navigation', executable='cone_navigator',
             name='cone_navigator', parameters=[params], output='screen',
             prefix=LaunchConfiguration('navigator_prefix'),
             condition=IfCondition(LaunchConfiguration('navigator'))),
    ])
