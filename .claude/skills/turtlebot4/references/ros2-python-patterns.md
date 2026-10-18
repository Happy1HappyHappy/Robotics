# ROS 2 Python patterns (CS 5335, Days 2–4)

These are the course's slide patterns, rewritten as valid Jazzy `rclpy` code in this repo's
style. The slides are pseudocode. When copying from them, fix these known errors:

| Slide text | Correct |
| --- | --- |
| `myLock = Lock()` | `import threading` then `threading.Lock()` |
| `if count = 0:` | `if count == 0:` |
| `print 'failed to get transform'` | `self.get_logger().warn(...)` (Python 3) |
| YAML key `ros_parameters` / `ros-parameters` | `ros__parameters` (two underscores) |
| `--paramsfile` | `--params-file` |
| `sudo apt install python-numpy python-opencv` | `sudo apt install python3-numpy python3-opencv` |
| `cv2.normalize(mat, NONE, ...)` | `cv2.normalize(mat, None, ...)` |
| curly quotes `‘ ’ “ ”` | straight quotes `'` |

## Contents

- [Node skeleton](#node-skeleton)
- [Publisher: velocity commands](#publisher-velocity-commands)
- [Subscriber: odometry](#subscriber-odometry)
- [Timer](#timer)
- [Services](#services)
- [Actions](#actions)
- [Defining interfaces](#defining-interfaces)
- [Parameters](#parameters)
- [Launch files](#launch-files)
- [Logging](#logging)
- [TF2](#tf2)
- [Multithreading and executors](#multithreading-and-executors)

## Node skeleton

```python
"""
One-line summary of what this node does
"""

import rclpy
from rclpy.node import Node


class MyNode(Node):
    """
    What the node is responsible for
    """
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('my_node started')


def main(args=None):
    """
    Entry point: create the node and spin until Ctrl-C
    """
    rclpy.init(args=args)
    node = MyNode()
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

Register the node in `setup.py`, then rebuild:

```python
entry_points={
    'console_scripts': [
        'my_node = mypkg.my_node:main',     # executable = package.module:function
    ],
},
```

Run it with `ros2 run mypkg my_node`.

## Publisher: velocity commands

```python
from geometry_msgs.msg import TwistStamped

# in __init__
self.cmd_pub_ = self.create_publisher(TwistStamped, 'cmd_vel', 10)   # queue depth 10

def publish_cmd(self, linear: float, angular: float):
    """
    Publish a velocity command
    linear: m/s along +x (forward)
    angular: rad/s about +z (+ = left/CCW)
    """
    msg = TwistStamped()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.twist.linear.x = float(linear)
    msg.twist.angular.z = float(angular)
    self.cmd_pub_.publish(msg)
```

Check whether the robot is listening with `self.cmd_pub_.get_subscription_count()` (see
`day2pkg/node1.py`). From the command line, see `Week1/ros_twist.bash`:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"
```

## Subscriber: odometry

```python
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

# in __init__
self.latest_odom_ = None
self.odom_sub_ = self.create_subscription(
    Odometry, 'odom', self.odom_callback, qos_profile_sensor_data)

def odom_callback(self, msg: Odometry):
    """
    Keep the latest odometry message (single writer, so no lock needed)
    """
    self.latest_odom_ = msg
```

The course default queue depth is 10 (`create_subscription(Type, 'topic', callback, 10)`).
For the robot's sensor topics, this repo uses `qos_profile_sensor_data`, a best-effort profile
that works with both best-effort and reliable publishers. Keep callbacks short: store or
compute and return.

## Timer

```python
# in __init__: call self.timer_callback every 0.1 s (fractions allowed)
self.timer_ = self.create_timer(0.1, self.timer_callback)

def timer_callback(self):
    ...

self.timer_.cancel()      # stop
self.timer_.reset()       # restart a canceled timer
```

The slides note that the Jazzy docs mix seconds and nanoseconds:
`create_timer` takes seconds, while `timer.timer_period_ns` is in nanoseconds.

## Services

Service definition (`srv/MyService.srv`): the request fields, then `---`, then the response
fields.

**Server**

```python
from my_interfaces.srv import MyService

# in __init__
self.service_ = self.create_service(MyService, 'my_service', self.handle_request)

def handle_request(self, request: MyService.Request, response: MyService.Response):
    """
    Fill in the response from the request and return it
    """
    response.result = request.a + request.b
    return response
```

**Client, asynchronous (use this inside a running node)**

```python
self.client_ = self.create_client(MyService, 'my_service')
if not self.client_.wait_for_service(timeout_sec=5.0):
    self.get_logger().warn('my_service not available')

request = MyService.Request()
request.a = 1
future = self.client_.call_async(request)
future.add_done_callback(self.response_callback)

def response_callback(self, future):
    response = future.result()
```

**Client, synchronous**: `rclpy.spin_until_future_complete(self, future)` followed by
`future.result()`. Only use this where nothing else spins the node, e.g. a CLI loop like
`executive.py`. Calling it inside a callback while an executor is already spinning deadlocks.

## Actions

Action definition (`action/MyAction.action`): goal, then `---`, result, then `---`, feedback.
The repo's `Drive.action` is an example.

**Server, with cancel**: the execute loop has to keep running while a cancel request
arrives, so the server needs a `MultiThreadedExecutor`.

```python
import time

from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from day3_interfaces.action import Drive

# in __init__
self.action_server_ = ActionServer(
    self, Drive, 'drive',
    goal_callback=self.goal_callback,
    cancel_callback=self.cancel_callback,
    execute_callback=self.execute_callback,
    callback_group=ReentrantCallbackGroup(),
)

def goal_callback(self, goal: Drive.Goal):
    """
    Validate the request. Reject bad input here, not halfway through execution
    """
    if goal.distance < 0.0:
        self.get_logger().warn(f'Reject: distance={goal.distance}')
        return GoalResponse.REJECT
    return GoalResponse.ACCEPT

def cancel_callback(self, goal_handle):
    """
    Decide whether to accept a cancel request
    """
    return CancelResponse.ACCEPT

def execute_callback(self, goal_handle):
    """
    Do the long-running work, publishing feedback and checking for cancel
    """
    while not done:
        if goal_handle.is_cancel_requested:
            self.publish_cmd(0.0, 0.0)          # stop first
            goal_handle.canceled()
            return Drive.Result()
        # ... publish a command, then optionally:
        # feedback = Drive.Feedback(); goal_handle.publish_feedback(feedback)
        time.sleep(period)
    self.publish_cmd(0.0, 0.0)
    goal_handle.succeed()
    return Drive.Result()

# in main()
executor = MultiThreadedExecutor()
executor.add_node(node)
executor.spin()
```

Cancel from a terminal with `robotics_ws/src/day3pkg/cancel.sh`:
`ros2 service call /drive/_action/cancel_goal action_msgs/srv/CancelGoal "{}"`.

**Client, asynchronous chain**

```python
from rclpy.action import ActionClient

self.client_ = ActionClient(self, Drive, 'drive')

def send_goal(self, goal: Drive.Goal):
    self.client_.wait_for_server()
    future = self.client_.send_goal_async(goal, feedback_callback=self.feedback_callback)
    future.add_done_callback(self.goal_response_callback)

def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback

def goal_response_callback(self, future):
    self.goal_handle_ = future.result()
    if not self.goal_handle_.accepted:
        self.get_logger().info('Goal rejected')
        return
    self.goal_handle_.get_result_async().add_done_callback(self.result_callback)

def result_callback(self, future):
    status = future.result().status             # action_msgs.msg.GoalStatus.STATUS_*
    result = future.result().result

def cancel(self):
    self.goal_handle_.cancel_goal_async().add_done_callback(self.cancel_done_callback)

def cancel_done_callback(self, future):
    canceled = future.result().goals_canceling  # empty list = nothing was canceled
```

The synchronous version (`spin_until_future_complete` on the goal and result futures) is what
`executive.py` uses for its blocking command loop.

## Defining interfaces

Create a package just for interfaces:

```bash
cd robotics_ws/src
ros2 pkg create my_interfaces            # ament_cmake by default
cd my_interfaces && rm -r src include && mkdir msg srv action
```

`package.xml`, after `<buildtool_depend>ament_cmake</buildtool_depend>` (Day 2 slides):

```xml
<build_depend>rosidl_default_generators</build_depend>
<depend>action_msgs</depend>                    <!-- only needed for .action files -->
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

The repo's `day3_interfaces` uses `<buildtool_depend>rosidl_default_generators</buildtool_depend>`
instead, which also works.

`CMakeLists.txt`, before `ament_package()`:

```cmake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MyMessage.msg"
  "srv/MyService.srv"
  "action/MyAction.action"
)
ament_export_dependencies(rosidl_default_runtime)
```

Field types: `bool`, `int32`, `float64` (Python `float`), `string`, `type[]` (list), or another
message type for nesting, e.g. `geometry_msgs/Pose`. Add units and sign conventions as
comments, as `Drive.action` does.

In each node package that uses them: add `<depend>my_interfaces</depend>` to `package.xml` and
`from my_interfaces.msg import MyMessage` in Python. Rebuild the interfaces package first, then
check with `ros2 interface show my_interfaces/msg/MyMessage`.

## Parameters

```python
self.declare_parameter('linear_speed', 0.15)      # m/s; type comes from the default
speed = self.get_parameter('linear_speed').value
```

```bash
ros2 run day3pkg driver --ros-args -p linear_speed:=0.1 -p rate_hz:=20.0
ros2 run day3pkg driver --ros-args --params-file src/day3pkg/config/driver.yaml
ros2 param dump /driving_node            # current values, in YAML
ros2 param set /driving_node linear_speed 0.1
```

YAML parameter file (keep it in `<pkg>/config/`):

```yaml
/driving_node:
  ros__parameters:
    linear_speed: 0.15
    angular_speed: 0.5
```

One file can hold several nodes' sections. Use `-r __node:=<name>` to pick the node name the
section applies to.

## Launch files

`<pkg>/launch/<name>_launch.py` (see `day3pkg/launch/day3_launch.py`):

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Start the driver with parameters from config/driver.yaml
    """
    params = os.path.join(get_package_share_directory('day3pkg'), 'config', 'driver.yaml')
    ld = LaunchDescription()
    ld.add_action(Node(
        package='day3pkg',
        executable='driver',
        name='driving_node',
        output='screen',
        parameters=[params],            # or [{'linear_speed': 0.15}]
    ))
    return ld
```

Install `launch/` and `config/` in `setup.py` `data_files`, e.g.
`(os.path.join('share', package_name, 'config'), glob('config/*.yaml'))`. Add `launch` and
`launch_ros` as `<exec_depend>` in `package.xml`. Start it with
`ros2 launch day3pkg day3_launch.py`.

## Logging

`self.get_logger().debug/info/warn/error/fatal('text')`

| Level | Use |
| --- | --- |
| fatal | Must crash noisily |
| error | Must stop; a restart might fix it |
| warn | Keeps running, but the operator should look |
| info | Progress: startup, shutdown, goal accepted |
| debug | Detail for developers |

Use `throttle_duration_sec=1.0` for messages inside loops. Include units and the values that
explain a decision, e.g. `f'Reject: distance={d} m'`.

## TF2

`lookup_transform('A', 'B', time)` returns the pose of frame B expressed in frame A. That is
the course's ^A T_B, which maps points from B into A.

```python
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# in __init__
self.tf_broadcaster_ = TransformBroadcaster(self)
self.tf_buffer_ = Buffer()
self.tf_listener_ = TransformListener(self.tf_buffer_, self)

# publish a transform
t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'odom'          # parent
t.child_frame_id = 'my_frame'       # child
t.transform.translation.x = x
t.transform.rotation.z = math.sin(yaw / 2.0)   # 2D yaw -> quaternion (x = y = 0)
t.transform.rotation.w = math.cos(yaw / 2.0)
self.tf_broadcaster_.sendTransform(t)

# look one up (Time() = latest available)
try:
    t = self.tf_buffer_.lookup_transform('odom', 'base_link', rclpy.time.Time())
except TransformException as ex:
    self.get_logger().warn(f'No transform odom -> base_link: {ex}')
```

Converting quaternions is the course's (Day 4) `tf_transformations`. It stores quaternions as
`(qx, qy, qz, qw)`:

```python
from tf_transformations import euler_from_quaternion, quaternion_from_euler

q = t.transform.rotation
roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
```

If `python3 -c "import tf_transformations"` fails, install it
(`sudo apt install ros-jazzy-tf-transformations`) and add
`<depend>tf_transformations</depend>` to `package.xml`. The existing `driver.py` uses its own
`yaw_from_quaternion()` helper for 2D, which gives the same yaw with no extra package.
To inspect the frame tree: `ros2 run tf2_tools view_frames`.

## Multithreading and executors

- Every callback belongs to a callback group. The default group is
  `MutuallyExclusiveCallbackGroup`, so only one of its callbacks runs at a time.
  `ReentrantCallbackGroup` lets its callbacks run concurrently.
- The default `SingleThreadedExecutor` runs one callback at a time across all groups. To get
  real concurrency, use `MultiThreadedExecutor`.
- Pattern used in `driver.py`: the action server goes in a `ReentrantCallbackGroup`, the odom
  subscription stays in the default group, and a `MultiThreadedExecutor` runs both. As a
  result, odometry and cancel requests keep arriving while `execute_callback` loops.
- Shared data written by more than one thread needs a lock. Use `with self.lock_:`, never bare
  `acquire()`/`release()`. A single writer with readers that only read a reference (like
  `latest_odom_`) does not need one.
- With several locks, always acquire them in the same order, to avoid deadlock.
- Keep critical sections small. A worker thread that dies while a status thread keeps
  reporting "OK" is the slides' "zombie state". Log errors from worker threads.
