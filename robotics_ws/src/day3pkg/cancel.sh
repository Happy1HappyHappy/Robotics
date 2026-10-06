#!/bin/bash
ros2 service call /drive/_action/cancel_goal action_msgs/srv/CancelGoal "{}"

# This script cancels the current goal for the /drive action server.
# before executing this script, make sure the /drive action server is running.
# chmod +x cancel.sh
# ./cancel.sh