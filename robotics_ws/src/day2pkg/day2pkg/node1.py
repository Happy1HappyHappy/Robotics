"""
This module defines Node1, a ROS2 node for controlling a robot's velocity
using TwistStamped messages
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

TOPIC = '/cmd_vel'      # topic for velocity commands
RATE_HZ = 20.0          # publishing rate in Hz
LIN_SPEED = 0.10        # linear speed in m/s
ANG_SPEED = 0.50        # angular speed in rad/s
PAUSE_SEC = 0.5         # pause duration in seconds

LIN_CAL = 1.0           # distance calibration factor (scales segment duration)
ANG_CAL = 1.0           # angle calibration factor (scales segment duration)


# === Task 3: Creating Twists ===
def CreateLinearTwist(x):
    """
    Create a TwistStamped message with a linear velocity along the x-axis

    Args:
        x (float): Linear velocity in m/s

    Returns:
        TwistStamped: The created TwistStamped message
    """
    msg = TwistStamped()
    msg.twist.linear.x = float(x)   # float conversion for safety
    return msg


def CreateAngularTwist(z):
    """
    Create a TwistStamped message with an angular velocity around the z-axis

    Args:
        z (float): Angular velocity in rad/s

    Returns:
        TwistStamped: The created TwistStamped message
    """
    msg = TwistStamped()
    msg.twist.angular.z = float(z)   # float conversion for safety
    return msg


def CreateArcTwist(x, z):
    """
    Create a TwistStamped message with both linear and angular velocities

    Args:
        x (float): Linear velocity in m/s
        z (float): Angular velocity in rad/s

    Returns:
        TwistStamped: The created TwistStamped message
    """
    msg = CreateLinearTwist(x)
    msg.twist.angular.z = float(z)
    return msg


# === Node1 Definition ===
class Node1(Node):
    """
    Node1 is a ROS2 node that publishes TwistStamped messages to control
    a robot's velocity
    """
    def __init__(self):
        """
        Initialize the Node1 instance
        """
        super().__init__('node1')
        self.publisher_ = self.create_publisher(TwistStamped, TOPIC, 10)
        self.segments = []          # [(msg, duration_sec, label), ...]
        self.seg_start = None       # start time of the current segment
        self.done = False   # indicate whether the node has finished sending msgs
        # self.counter = 0    # count the number of messages sent for Task2
        self.timer_ = self.create_timer(
                        1.0 / RATE_HZ, self.callback)  # register a timer callback
        self.get_logger().info('node1 started')

    def send(self, msg):
        """
        Send a TwistStamped message with the current timestamp

        Args:
            msg (TwistStamped): The message to be sent
        """
        msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'base_link'   # optional, default as robot base frame
        self.publisher_.publish(msg)

    def add_segment(self, msg, duration, label):
        """
        Add a segment to the list of motion segments

        Args:
            msg (TwistStamped): The message representing the motion segment
            duration (float): Duration of the segment in seconds
            label (str): Label for the segment
        """
        self.segments.append((msg, duration, label))

    # === Task 5: Driving Functions ===
    def DriveStraight(self, d):
        """
        Drive straight for a specified distance

        Args:
            d (float): Distance to drive in meters
        """
        v = LIN_SPEED if d >= 0 else -LIN_SPEED
        duration = abs(d) / LIN_SPEED * LIN_CAL
        self.add_segment(
            CreateLinearTwist(v), duration, f'DriveStraight {d}')

    def DriveCircle(self, r):
        """
        Drive in a circle with a specified radius

        Args:
            r (float): Radius of the circle in meters
        """
        self.DriveArc(r, 360)

    def DriveArc(self, r, d):
        """
        Drive along an arc with a specified radius and angle
        If the radius is approximately zero, turn in place instead

        Args:
            r (float): Radius of the arc in meters
            d (float): Angle to drive along the arc in degrees
        """
        if r < 0:
            raise ValueError(f'DriveArc: radius must be >= 0, got {r}')
        if r < 1e-6:                     # r ≈ 0, turn in place
            self.Turn(d)
            return

        arc_len = r * math.radians(abs(d))
        w = (LIN_SPEED / r) * (1 if d >= 0 else -1)   # v = ω r
        t = arc_len / LIN_SPEED * LIN_CAL
        self.add_segment(CreateArcTwist(LIN_SPEED, w), t, f'DriveArc({r}, {d})')

    def Turn(self, angle_deg):
        """
        Turn the robot by a specified angle

        Args:
            angle_deg (float): Angle to turn in degrees
        """
        w = ANG_SPEED if angle_deg >= 0 else -ANG_SPEED
        t = math.radians(abs(angle_deg)) / ANG_SPEED * ANG_CAL
        self.add_segment(CreateAngularTwist(w), t, f'Turn({angle_deg})')

    def Pause(self, sec=PAUSE_SEC):
        """
        Pause the robot for a specified duration

        Args:
            sec (float): Duration to pause in seconds
        """
        self.add_segment(TwistStamped(), sec, f'Pause {sec}')

    # === Callback Function ===
    def callback(self):
        """
        Callback function for handling robot control messages
        """
        if self.done:
            return

        if self.publisher_.get_subscription_count() == 0:
            self.get_logger().info(
                # print a message indicating waiting for robot once per second
                'Waiting for robot..', throttle_duration_sec=1.0)
            return

        if not self.segments:
            self.stop()     # stop the robot if there are no more segments
            return

        msg, duration, label = self.segments[0]
        now = self.get_clock().now()
        if self.seg_start is None:
            self.seg_start = now
            self.get_logger().info(f'Start {label}: {duration:.2f} s')

        elapsed = (now - self.seg_start).nanoseconds / 1e9  # convert elapsed time to seconds
        if elapsed >= duration:
            self.segments.pop(0)
            self.seg_start = None
            self.send(TwistStamped())   # send zero velocity before moving to the next segment
        else:
            self.send(msg)   # send the current segment's velocity command

        # === Task 2: Drive Your Turtlebot -> kept for reference, not used ===
        # if self.counter < 3:
        #     msg = TwistStamped()
        #     msg.twist.linear.x = 0.1
        #     self.send(msg)
        #     self.counter += 1
        #     self.get_logger().info(f'Callback {self.counter}')
        # else:
        #     # in case robot not get the last velocity command, send three times
        #     for _ in range(3):
        #         # send zero velocity message to stop the robot
        #         self.send(TwistStamped())
        #     self.get_logger().info('Robot stopped')
        #     self.timer_.cancel()    # stop the timer to prevent further callbacks
        #     self.done = True        # indicate that the node has finished sending messages

    def stop(self):
        """
        Stop the robot by sending zero velocity commands multiple times
        """
        for _ in range(3):              # send multiple times to ensure the robot stops
            self.send(TwistStamped())
        self.get_logger().info('Path finished, stopping')
        self.done = True


def main(args=None):
    """
    Main function to initialize the ROS node and run the robot control loop
    """
    rclpy.init(args=args)
    node = Node1()

    # Task 4 & 5:
    # node.DriveStraight(0.5)
    # node.DriveStraight(1.0)
    # node.DriveCircle(0.5)
    # node.DriveArc(0.5, 90)

    # Task 6: Driving a D-shaped path
    node.DriveStraight(1.0)
    node.Pause()
    node.Turn(-90)          # turn right 90 degrees
    node.Pause()
    node.DriveArc(0.5, -180)  # drive a right semicircle with radius 0.5 m
    node.Pause()
    node.Turn(-90)          # turn right 90 degrees

    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        try:
            node.send(TwistStamped())
        except Exception:
            pass

    # Tear down the node and shutdown ROS
    time.sleep(0.2)      # wait for the last message to be sent
    node.destroy_node()  # destroy the node to clean up resources
    if rclpy.ok():
        rclpy.shutdown()     # shutdown the ROS client library


if __name__ == '__main__':
    main()
