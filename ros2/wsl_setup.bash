# Source this file in WSL to use RUKA3 with the Raspberry Pi discovery server.
ruka3_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/jazzy/setup.bash
source "$ruka3_root/build/ros2-install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-1}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -z "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -z "${FASTDDS_DEFAULT_PROFILES_FILE:-}" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$ruka3_root/ros2/config/fastdds-wsl.xml"
fi
unset ruka3_root
