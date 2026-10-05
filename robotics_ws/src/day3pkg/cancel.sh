#!/bin/bash
ros2 service call /drive/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
