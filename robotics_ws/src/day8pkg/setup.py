from setuptools import find_packages, setup

package_name = 'day8pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='robo@todo.todo',
    description='Project 8: drive a path clicked in RViz with Nav2 (TurtleBot 4 Navigator)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'create_path = day8pkg.create_path:main',
            'point_path = day8pkg.point_path:main',
        ],
    },
)
