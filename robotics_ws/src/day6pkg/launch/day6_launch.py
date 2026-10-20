"""
Launch file for the day6pkg package: the Grid Node plus day3pkg's Driving and Executive nodes
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for Project 6
    'ros2 launch' calls this function to get the launch description
    """
    ld = LaunchDescription()

    mapper = Node(
        package='day6pkg',
        executable='mapper',
        name='grid_node',
        output='screen',
    )

    # day3pkg's own launch file: Driving Node + Executive in an xterm
    # (quitting the Executive stops the whole launch, including the Grid Node)
    day3 = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('day3pkg'), 'launch', 'day3_launch.py')))

    # add_action adds a node (or an included launch file) to the launch process
    ld.add_action(mapper)
    ld.add_action(day3)
    return ld
