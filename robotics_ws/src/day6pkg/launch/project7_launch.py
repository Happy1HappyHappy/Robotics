"""
Launch file for Project 7: the Grid Node, the Landmark Node and the Viewer Node, with
config/project7.yaml
ros2 launch day6pkg project7_launch.py
Each launch is one run, named after its start time (run_id:= names it yourself); all nodes
name their files after it, like log files: logs/<run>_grid.npz, logs/<run>_landmarks.npz,
result/grid/<run>_grid_*.png, saved every save_period seconds and on Ctrl-C
draw:=false leaves the Viewer Node out: the other two run the same, nothing is drawn
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from day6pkg.helpers.files import run_name


def generate_launch_description():
    """
    Generate launch description for Project 7
    'ros2 launch' calls this function to get the launch description
    """
    ld = LaunchDescription()
    config = os.path.join(get_package_share_directory('day6pkg'), 'config', 'project7.yaml')
    # the run's name, read as text even if it looks like a number or a date
    run_id = ParameterValue(LaunchConfiguration('run_id'), value_type=str)

    ld.add_action(DeclareLaunchArgument('run_id', default_value=run_name(),
                                        description='name of this run (default: start time)'))
    ld.add_action(DeclareLaunchArgument('draw', default_value='true',
                                        description='start the Viewer Node'))
    # parameters: the YAML file first, then the run's name on top of it
    for executable, name, condition in (('grid_node', 'grid_node', None),
                                        ('landmark_node', 'landmark_node', None),
                                        ('viewer_node', 'viewer_node',
                                         IfCondition(LaunchConfiguration('draw')))):
        ld.add_action(Node(
            package='day6pkg',
            executable=executable,
            name=name,
            output='screen',
            parameters=[config, {'run_id': run_id}],
            condition=condition,
        ))
    return ld
