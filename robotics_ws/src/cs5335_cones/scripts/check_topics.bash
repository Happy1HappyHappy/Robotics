#!/bin/bash
# Copyright 2026 Claire Liu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Checks that every topic the cone driver uses works, before running it: the camera, LiDAR,
# odometry and hazards are published and their data arrives here, and the robot's base
# listens on cmd_vel (with --move, also that it turns when told to).
#
# Run from the workspace root (robotics_ws), in a terminal with the robot's ROS settings:
#   bash src/cs5335_cones/scripts/check_topics.bash           # no motion
#   bash src/cs5335_cones/scripts/check_topics.bash --move    # also turns the robot ~17 deg
# Exits with 0 if every check passes, 1 if any fails.

set -u

move=false
if [[ "${1-}" == "--move" ]]; then
    move=true
elif [[ $# -gt 0 ]]; then
    echo "Usage: $0 [--move]"
    exit 2
fi

failed=0
ok() { echo "  OK    $*"; }
bad() { echo "  FAIL  $*"; failed=1; }

if ! command -v ros2 > /dev/null; then
    echo "ros2 not found: run 'source /opt/ros/jazzy/setup.bash' first"
    exit 1
fi

echo "== ROS settings (must be the same in every terminal, and match the robot's)"
for v in ROS_DOMAIN_ID RMW_IMPLEMENTATION ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT \
         FASTDDS_BUILTIN_TRANSPORTS; do
    echo "  $v=${!v-(not set)}"
done

# topic_info TOPIC: what ros2 topic info prints for it
topic_info() {
    timeout -k 3 10 ros2 topic info "$1" 2>&1
}

# count KIND INFO: the number after "KIND count:" in ros2 topic info's output, or 0
count() {
    local n
    n=$(echo "$2" | awk -v field="$1 count:" 'index($0, field) == 1 {print $NF}')
    echo "${n:-0}"
}

# check_publisher TOPIC TYPE: the topic exists with this type and has a publisher
check_publisher() {
    local info pubs
    info=$(topic_info "$1")
    if ! echo "$info" | grep -q "^Type: $2"; then
        bad "$1: not found as $2 (${info%%$'\n'*}); check the ROS settings"
        return 1
    fi
    pubs=$(count Publisher "$info")
    if [[ "$pubs" -lt 1 ]]; then
        bad "$1: no publisher; the robot's program for it is not running"
        return 1
    fi
    ok "$1: $pubs publisher(s)"
}

# check_rate TOPIC: messages arrive within 6 s, and how often
check_rate() {
    local rate
    # SIGINT rather than timeout's SIGTERM, so ros2 (Python) still writes out what it printed
    rate=$(PYTHONUNBUFFERED=1 timeout -s INT -k 3 6 ros2 topic hz "$1" 2>/dev/null |
           awk '/average rate:/ {r = $3} END {print r}')
    if [[ -z "$rate" ]]; then
        bad "$1: published, but no message arrived here in 6 s (Wi-Fi too slow?)"
        return 1
    fi
    ok "$1: $rate Hz"
}

# odom_yaw: the robot's yaw from one odom message, in degrees (on a flat floor 2 atan2(z, w))
odom_yaw() {
    PYTHONUNBUFFERED=1 timeout -s INT -k 3 5 ros2 topic echo --once /odom 2>/dev/null |
        awk '/orientation:/ {o = 1; next}
             o && $1 == "z:" {z = $2}
             o && $1 == "w:" {w = $2; exit}
             END {if (w == "") exit 1; printf "%.1f\n", 2 * atan2(z, w) * 57.29578}'
}

echo "== Sensor topics (up to 6 s each)"
for entry in "/oakd/rgb/preview/image_raw sensor_msgs/msg/Image" \
             "/scan sensor_msgs/msg/LaserScan" \
             "/odom nav_msgs/msg/Odometry"; do
    read -r topic type <<< "$entry"
    check_publisher "$topic" "$type" && check_rate "$topic"
done
# Hazard messages may not come while there is no hazard, so only the publisher is checked
check_publisher /hazard_detection irobot_create_msgs/msg/HazardDetectionVector

echo "== cmd_vel"
info=$(topic_info /cmd_vel)
if ! echo "$info" | grep -q '^Type: geometry_msgs/msg/TwistStamped'; then
    bad "/cmd_vel: not found as geometry_msgs/msg/TwistStamped (${info%%$'\n'*})"
elif [[ "$(count Subscription "$info")" -lt 1 ]]; then
    bad "/cmd_vel: nothing subscribes; the robot's base (Create 3) is not connected"
else
    ok "/cmd_vel: $(count Subscription "$info") subscriber(s)"
fi

if $move; then
    echo "== Turning the robot left for 1 s (about 17 degrees)"
    before=$(odom_yaw)
    if [[ -z "$before" ]]; then
        bad "could not read the yaw from /odom"
    else
        timeout -k 3 15 ros2 topic pub -r 10 -t 10 /cmd_vel geometry_msgs/msg/TwistStamped \
            "{twist: {angular: {z: 0.3}}}" > /dev/null
        timeout -k 3 10 ros2 topic pub -r 10 -t 3 /cmd_vel geometry_msgs/msg/TwistStamped \
            "{}" > /dev/null
        sleep 1
        after=$(odom_yaw)
        if [[ -z "$after" ]]; then
            bad "could not read the yaw from /odom after the turn"
        else
            turned=$(awk -v a="$after" -v b="$before" 'BEGIN {
                d = a - b; while (d > 180) d -= 360; while (d <= -180) d += 360
                printf "%.1f", d}')
            if awk -v d="$turned" 'BEGIN {exit !(d > 5)}'; then
                ok "turned $turned deg"
            else
                bad "turned only $turned deg: the commands on /cmd_vel did not reach the base"
            fi
        fi
    fi
fi

echo
if [[ $failed -eq 0 ]]; then
    echo "All checks passed"
else
    echo "Some checks failed (see FAIL above)"
fi
exit $failed
