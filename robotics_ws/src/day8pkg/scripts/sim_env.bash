# ROS settings for every simulator terminal. Source it, do not run it:
#   source ~/Robotics/robotics_ws/src/day8pkg/scripts/sim_env.bash
# The simulator runs entirely on this computer, so it must not use the real robot's discovery
# server. A ROS_DOMAIN_ID that nobody else nearby uses keeps other people's simulators and robots
# out. (ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST is not used: with the simulator's 60-odd
# programs, the later ones, such as map_server and the lifecycle managers, never find each other.)
source /opt/ros/jazzy/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTDDS_BUILTIN_TRANSPORTS ROS_LOCALHOST_ONLY \
    ROS_AUTOMATIC_DISCOVERY_RANGE
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=73   # any number from 1 to 101 that nobody nearby uses
# This workspace (robotics_ws), three levels up from this file, if it has been built
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ -f "$ws/install/setup.bash" ]; then
    source "$ws/install/setup.bash"
else
    echo "robotics_ws is not built yet: cd $ws && colcon build --packages-select day8pkg"
fi
echo "Simulator settings: ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
