"""
Launch file for Project 7: the Grid Node, the Landmark Node and the Viewer Node, with
config/project7.yaml
ros2 launch day6pkg project7_launch.py run:=run_a    (then run:=run_b for the way back)
draw:=false leaves the Viewer Node out: the other two run the same, nothing is drawn
All of them save to the run's folder when the launch is stopped with Ctrl-C
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for Project 7
    'ros2 launch' calls this function to get the launch description
    """
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('day6pkg'), 'config', 'project7.yaml')
    run = LaunchConfiguration('run')    # the folder this run is saved to

    ld.add_action(DeclareLaunchArgument('run', default_value='run_a',
                                        description='folder this run is saved to'))
    ld.add_action(DeclareLaunchArgument('draw', default_value='true',
                                        description='start the Viewer Node'))
    # parameters: the YAML file first, then output_dir from the run argument on top of it
    for executable, name, condition in (('grid_node', 'grid_node', None),
                                        ('landmark_node', 'landmark_node', None),
                                        ('viewer_node', 'viewer_node',
                                         IfCondition(LaunchConfiguration('draw')))):
        ld.add_action(Node(
            package='day6pkg',
            executable=executable,
            name=name,
            output='screen',
            parameters=[config, {'output_dir': run}],
            condition=condition,
        ))
    return ld
