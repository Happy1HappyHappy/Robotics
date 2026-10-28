"""
Launch file for Project 7: the Grid Node, the Landmark Node, the Planner Node and the
Viewer Node, with config/project7.yaml
ros2 launch day6pkg project7_launch.py
Each launch is one run, named after its start time (2026-10-27_16-40-12), so no two runs
share a name; all nodes name their files after it, like log files, in logs/ and
result/grid/: <run>_*_latest.* every save_period seconds while driving, <run>_*_final.* on
Ctrl-C
draw:=false leaves the Viewer Node out: the others run the same, nothing is drawn
plan:=false leaves the Planner Node out; goal_x:=<m> goal_y:=<m> replace its goal in the
YAML file
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from day6pkg.helpers.files import new_run_name


def nodes(context):
    """
    The run's name and its nodes, once the launch arguments are known
    """
    run = new_run_name()
    draw = LaunchConfiguration('draw').perform(context).lower() in ('true', '1', 'yes')
    plan = LaunchConfiguration('plan').perform(context).lower() in ('true', '1', 'yes')
    config = os.path.join(get_package_share_directory('day6pkg'), 'config', 'project7.yaml')
    # parameters: the YAML file first, then the run's name on top of it, read as text even
    # though it looks like a date
    parameters = [config, {'run_id': ParameterValue(run, value_type=str)}]
    # the Planner Node's goal (m): goal_x:= and goal_y:= if given, else the YAML file's
    goal = {name: float(value) for name in ('goal_x', 'goal_y')
            if (value := LaunchConfiguration(name).perform(context))}
    executables = (['grid_node', 'landmark_node'] + (['planner_node'] if plan else []) +
                   (['viewer_node'] if draw else []))
    return [LogInfo(msg=f'Run {run}')] + [
        Node(package='day6pkg', executable=name, name=name, output='screen',
             parameters=parameters + ([goal] if name == 'planner_node' else []))
        for name in executables]


def generate_launch_description():
    """
    Generate launch description for Project 7
    'ros2 launch' calls this function to get the launch description
    """
    ld = LaunchDescription()
    ld.add_action(DeclareLaunchArgument('draw', default_value='true',
                                        description='start the Viewer Node'))
    ld.add_action(DeclareLaunchArgument('plan', default_value='true',
                                        description='start the Planner Node'))
    ld.add_action(DeclareLaunchArgument('goal_x', default_value='',
                                        description='Planner Node goal x, m (default: YAML)'))
    ld.add_action(DeclareLaunchArgument('goal_y', default_value='',
                                        description='Planner Node goal y, m (default: YAML)'))
    ld.add_action(OpaqueFunction(function=nodes))
    return ld
