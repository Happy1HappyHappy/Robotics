from setuptools import find_packages, setup

package_name = 'day7pkg'

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
    description='Project 7: landmarks (RANSAC corners) and A* path planning on occupancy grids',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'landmarks = day7pkg.find_landmarks:main',
            'plan = day7pkg.plan_path:main',
        ],
    },
)
