# ROS settings for every simulator terminal. Source it, do not run it:
#   source ~/Robotics/robotics_ws/src/day8pkg/scripts/sim_env.bash
# The simulator runs entirely on this computer, so it must not use the real robot's discovery
# server, and it keeps to this computer so that other people's simulators cannot mix in.
source /opt/ros/jazzy/setup.bash
unset ROS_DISCOVERY_SERVER ROS_SUPER_CLIENT FASTDDS_BUILTIN_TRANSPORTS ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=8
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# This workspace (robotics_ws), three levels up from this file, if it has been built
ws="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
if [ -f "$ws/install/setup.bash" ]; then
    source "$ws/install/setup.bash"
else
    echo "robotics_ws is not built yet: cd $ws && colcon build --packages-select day8pkg"
fi
echo "Simulator settings: ROS_DOMAIN_ID=$ROS_DOMAIN_ID, this computer only"
