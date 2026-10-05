from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    driver = Node(
        package='day3pkg',
        executable='driver',
        name='driving_node',
        output='screen',
        parameters=[
            {'linear_speed': 0.15},
            {'angular_speed': 0.5},
        ],
    )

    executive = Node(
        package='day3pkg',
        executable='executive',
        name='executive_node',
        prefix='xterm -e',      # gives the Executive its own terminal for input()
        on_exit=Shutdown(),     # when the Executive quits, stop the whole launch (incl. driver)
    )

    ld.add_action(driver)
    ld.add_action(executive)
    return ld
