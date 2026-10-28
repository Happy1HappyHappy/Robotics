import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'day6pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='robo@todo.todo',
    description='Project 6: occupancy grid from LiDAR',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'grid_node = day6pkg.nodes.grid_node:main',
            'landmark_node = day6pkg.nodes.landmark_node:main',
            'planner_node = day6pkg.nodes.planner_node:main',
            'viewer_node = day6pkg.nodes.viewer_node:main',
        ],
    },
)
