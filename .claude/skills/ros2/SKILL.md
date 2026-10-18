---
name: ros2
description: >
  ROS 2 Jazzy best practices for Python (rclpy) work in the CS 5335 workspace: nodes, packages,
  the colcon workspace, package.xml and setup.py, custom messages, services and actions, launch
  files, parameters, QoS, and debugging with the ros2 command line. Use when building or fixing
  ROS 2 nodes, packages, launch files or interfaces, when colcon build fails, or when topics do
  not connect. For anything involving the TurtleBot 4 or this repository, the turtlebot4 skill
  (based on the course slides) takes precedence.
---

# ROS 2 Development Skill

> Adapted from [arpitg1304/robotics-agent-skills](https://github.com/arpitg1304/robotics-agent-skills)
> (commit f9bc546, Apache-2.0) for CS 5335. **Modified**: removed everything that conflicts with
> the course slides (C++/rclcpp, lifecycle nodes, components, DDS and discovery changes, multiple
> workspaces, `--symlink-install`, the production checklist) and rewrote the examples for ROS 2
> Jazzy and the course conventions. The course rules are in the `turtlebot4` skill. Follow that
> skill wherever the two differ.

## Course scope

- ROS 2 **Jazzy** on Ubuntu 24.04, **Python only** (`rclpy`). Do not write C++ or `rclcpp`.
- **One** workspace, `robotics_ws/`, which is also the git repository.
- Python nodes live in `ament_python` packages. Messages, services and actions live in a
  separate `ament_cmake` interfaces package.
- Velocity commands on `/cmd_vel` are `geometry_msgs/msg/TwistStamped` on the course's
  TurtleBot 4, **not** `Twist`. Most online examples use `Twist`, and the robot ignores it. The
  `turtlebot4` skill has the robot-specific topics, actions and rules.
- Not part of the course, so do not introduce them: lifecycle (managed) nodes, components and
  composition, intra-process communication, DDS or discovery tuning, SROS2, micro-ROS, MoveIt 2.

## When to Use This Skill

- Writing or fixing a ROS 2 node in Python
- Workspace and `colcon build` problems, `package.xml`, `setup.py`
- Defining custom messages, services or actions
- Launch files and parameters
- Topics that do not connect (QoS), missing nodes or executables

## Node pattern (rclpy)

One node per program, one `Node` subclass per file (Day 2 slides):

```python
"""
Example node: publish the closest LiDAR range as text at a fixed rate
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ScanSummaryNode(Node):
    """
    Subscribe to /scan and publish a short summary on a timer
    """
    def __init__(self):
        super().__init__('scan_summary_node')

        # 1. parameters: the type comes from the default value
        self.declare_parameter('rate_hz', 2.0)    # Hz

        # 2. publishers and subscribers: queue depth 10, sensor QoS for robot sensors
        self.summary_pub_ = self.create_publisher(String, 'scan_summary', 10)
        self.latest_scan_ = None
        self.scan_sub_ = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos_profile_sensor_data)

        # 3. timer for periodic work
        rate_hz = self.get_parameter('rate_hz').value
        self.timer_ = self.create_timer(1.0 / rate_hz, self.timer_callback)
        self.get_logger().info(f'scan_summary_node started at {rate_hz} Hz')

    def scan_callback(self, msg: LaserScan):
        """
        Keep only the latest scan; the timer does the work
        """
        self.latest_scan_ = msg

    def timer_callback(self):
        """
        Publish the closest valid range
        """
        scan = self.latest_scan_
        if scan is None:
            return
        valid = [r for r in scan.ranges if scan.range_min <= r <= scan.range_max]
        msg = String()
        msg.data = f'closest={min(valid):.2f} m' if valid else 'no valid ranges'
        self.summary_pub_.publish(msg)


def main(args=None):
    """
    Entry point: create the node and spin until Ctrl-C
    """
    rclpy.init(args=args)
    node = ScanSummaryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
```

- Attributes that hold ROS handles end in `_` (`self.summary_pub_`), as in the slides and the
  repo.
- The slides end `main()` with `rclpy.shutdown()`. After Ctrl-C the context may already be shut
  down, so the repo uses `rclpy.try_shutdown()` (or `if rclpy.ok(): rclpy.shutdown()`).
- Changing a parameter while the node runs needs a parameter callback, which the slides call
  "beyond this class". Read parameters where you use them, or restart the node.

## QoS

- **Course default**: queue depth 10, e.g. `self.create_publisher(Type, 'topic', 10)`. That
  gives RELIABLE, VOLATILE, KEEP_LAST(10).
- **Robot sensor topics** (`/odom`, `/scan`, `/imu`, …): subscribe with
  `qos_profile_sensor_data`. It is best-effort, so it connects to either kind of publisher.

**Compatibility matrix**:

```
Publisher     Subscriber    Compatible?
RELIABLE      RELIABLE      ✅ Yes
RELIABLE      BEST_EFFORT   ✅ Yes
BEST_EFFORT   BEST_EFFORT   ✅ Yes
BEST_EFFORT   RELIABLE      ❌ NO — SILENT FAILURE
```

**Debugging**: if a topic is listed but your callback never runs, compare the
`Reliability` / `Durability` fields from `ros2 topic info /topic -v` on both sides.

## Environment and discovery

Use the course setup exactly. Do not change it:

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8          # must match the robot
```

- The laptop connects through the TurtleBot's Discovery Server (Computer Setup document,
  step 9). The TurtleBot 4 supports Discovery Server only with Fast DDS, so do **not** switch to
  CycloneDDS.
- Do **not** set `ROS_LOCALHOST_ONLY` or `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. The laptop
  would stop seeing the robot.
- When topics are missing, run `ros2 daemon stop; ros2 daemon start`.

## Launch files

Course style (Day 3 slides, see `day3pkg/launch/day3_launch.py`):

```python
"""
Launch the driver with its parameter file
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Called by 'ros2 launch' to get the nodes to start
    """
    params = os.path.join(get_package_share_directory('day3pkg'), 'config', 'driver.yaml')
    ld = LaunchDescription()
    ld.add_action(Node(
        package='day3pkg',
        executable='driver',
        name='driving_node',
        output='screen',
        parameters=[params],          # or [{'linear_speed': 0.15}]
    ))
    return ld
```

- Do not add a `namespace` to TurtleBot nodes. The robot's topics are not namespaced, so
  `cmd_vel` would become `/<namespace>/cmd_vel` and the robot would not move.
- Run it with `ros2 launch <package> <launch file>`. Install `launch/` (and `config/`) through
  `setup.py` `data_files`.

## Actions

To cancel an action while it runs, the server must be multithreaded (Day 3 slides): use a
**synchronous** `execute_callback` in a `ReentrantCallbackGroup`, spun by a
`MultiThreadedExecutor`. Stop the robot before reporting cancel.

```python
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from day3_interfaces.action import Drive


class DrivingNode(Node):
    """
    Serve the drive action
    """
    def __init__(self):
        super().__init__('driving_node')
        self.action_server_ = ActionServer(
            self, Drive, 'drive',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def goal_callback(self, goal: Drive.Goal):
        """
        Reject invalid goals before execution starts
        """
        return GoalResponse.REJECT if goal.distance < 0.0 else GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Accept every cancel request
        """
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Long-running work; checks for cancel on every step
        """
        for _ in range(10):
            if goal_handle.is_cancel_requested:
                # stop the robot here first
                goal_handle.canceled()
                return Drive.Result()
            time.sleep(0.1)       # one control step
        goal_handle.succeed()
        return Drive.Result()


def main(args=None):
    """
    Spin with a MultiThreadedExecutor so cancel requests are handled during execution
    """
    rclpy.init(args=args)
    node = DrivingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
```

The action client patterns are in the `turtlebot4` skill
(`references/ros2-python-patterns.md`).

## Build System

### Workspace and colcon

```bash
mkdir -p robotics_ws/src     # once: a workspace is a folder with src/
cd robotics_ws
colcon build                 # creates build/ install/ log/
source install/setup.bash
```

- `~/.bashrc` should contain `source /opt/ros/jazzy/setup.bash`, with the workspace's
  `install/setup.bash` below it.
- **After every change** (including editing a Python file): `colcon build`, then
  `source install/setup.bash`.
- Never source `setup.bash` from `build/`. Always use `install/`.

**Useful colcon flags**:

```bash
colcon build --packages-select my_pkg              # build only these packages
colcon build --packages-select day3_interfaces day3pkg   # interfaces first
colcon build --packages-up-to my_pkg               # a package and everything it depends on
colcon build --event-handlers console_direct+      # show build output live
rm -rf build/ install/ log/ && colcon build        # clean build
colcon test --packages-select my_pkg && colcon test-result --verbose
```

This workspace is built **without** `--symlink-install`. Do not add it: switching an existing
workspace to symlinks fails until `build/` and `install/` are deleted, and the course workflow
is to rebuild after each edit.

### Creating packages (Day 2 slides)

```bash
cd robotics_ws/src
# Python node package
ros2 pkg create my_pkg --build-type ament_python --dependencies rclpy --license Apache-2.0
# interfaces package (ament_cmake is the default), then make it interfaces-only
ros2 pkg create my_interfaces
cd my_interfaces && rm -r src include && mkdir msg srv action
```

Note that the Python sources end up in `src/my_pkg/my_pkg/` (the package name twice).

### Build types

```
ament_python    — Python node packages (no custom messages)
ament_cmake     — interfaces packages (msg/, srv/, action/)
```

### package.xml — Python node package

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_pkg</name>
  <version>0.0.0</version>
  <description>What the package does</description>
  <maintainer email="robo@todo.todo">robo</maintainer>
  <license>Apache-2.0</license>

  <!-- every package whose messages, services or actions you import -->
  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>my_interfaces</depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

List each dependency **once**. `<depend>` already covers build and exec, and colcon rejects a
`package.xml` that repeats the same package in `<depend>` and `<build_depend>`/`<exec_depend>`.
The build type comes from `<export><build_type>`.

### package.xml and CMakeLists.txt — interfaces package (Day 2 slides)

`package.xml`, after `<buildtool_depend>ament_cmake</buildtool_depend>`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<depend>action_msgs</depend>                <!-- only if the package has .action files -->
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

`CMakeLists.txt`, after `find_package(ament_cmake REQUIRED)` and before `ament_package()`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"
  "srv/GetPose.srv"
  "action/MoveForward.action"
  DEPENDENCIES geometry_msgs     # needed when fields use another package's types
)
ament_export_dependencies(rosidl_default_runtime)
```

If you use `DEPENDENCIES geometry_msgs`, also add `find_package(geometry_msgs REQUIRED)` and
`<depend>geometry_msgs</depend>`.

### setup.py — Python package

```python
from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robo',
    maintainer_email='robo@todo.todo',
    description='What the package does',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # executable = package.module:function   (no .py on the module)
            'my_node = my_pkg.my_node:main',
        ],
    },
)
```

`setup.cfg` is generated by `ros2 pkg create`. Leave it as is.

### Custom message, service, and action definitions

Names are UpperCamelCase. Each file contains only `type name` lines (Day 2–3 slides):

```
# msg/Detection.msg
std_msgs/Header header
string class_name
float32 confidence
geometry_msgs/Pose pose
```

```
# srv/GetPose.srv  — request, ---, response
string object_name
---
bool success
geometry_msgs/PoseStamped pose
```

```
# action/MoveForward.action  — goal, ---, result, ---, feedback
float64 distance   # meters
---
bool success
---
float64 remaining  # meters
```

Python types: `bool`→bool, `int32`→int, `float64`→float, `string`→str, `type[]`→list. Assign
`float()` values to float fields, because a Python `int` is rejected.

### Build troubleshooting

```bash
# "Could not find a package configuration file provided by X"
# -> the underlay is not sourced
source /opt/ros/jazzy/setup.bash

# Build succeeds but 'ros2 run' cannot find the package or executable
# -> overlay not sourced, or the console_scripts entry is wrong
source install/setup.bash
ros2 pkg list | grep my_pkg
ros2 pkg executables my_pkg

# Python changes not reflected
# -> rebuild that package and re-source
colcon build --packages-select my_pkg && source install/setup.bash

# Custom messages not found by Python nodes
# -> build the interfaces package first, re-source, and check <depend>my_interfaces</depend>
ros2 interface show my_interfaces/msg/MyMessage

# "Multiple packages with the same name"
colcon list | grep my_pkg

# package.xml rejected ("dependency ... is redundant")
# -> the same package is listed in <depend> and in <build_depend>/<exec_depend>
```

## Package structure

```
robotics_ws/                      # the one workspace (also the git repo folder)
└── src/
    ├── my_pkg/                   # ament_python node package
    │   ├── package.xml
    │   ├── setup.py
    │   ├── setup.cfg
    │   ├── resource/my_pkg
    │   ├── my_pkg/               # Python module (package name again)
    │   │   ├── __init__.py
    │   │   └── my_node.py
    │   ├── launch/my_launch.py
    │   ├── config/my_node.yaml
    │   └── test/                 # generated lint tests
    └── my_interfaces/            # ament_cmake interfaces package
        ├── package.xml
        ├── CMakeLists.txt
        ├── msg/  srv/  action/
```

## Debugging toolkit

```bash
# static (what was built)
ros2 interface list
ros2 interface show day3_interfaces/action/Drive

# dynamic (what is running now)
ros2 node list
ros2 node info /driving_node
ros2 topic list -t
ros2 topic info /odom -v               # QoS details
ros2 topic hz /odom
ros2 topic echo /cmd_vel
ros2 topic echo /odom --once
ros2 service list -t
ros2 action list -t
ros2 action info /drive

# parameters
ros2 param list /driving_node
ros2 param get /driving_node linear_speed
ros2 param dump /driving_node

# recording
ros2 bag record /odom /cmd_vel
ros2 bag info <bag folder>

# discovery
ros2 doctor
ros2 daemon stop; ros2 daemon start
```
