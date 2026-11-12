#!/usr/bin/env bash
# MoveIt and RViz in WSL; controller_manager stays on Raspberry Pi.
set -e
ruka3_script="$(readlink -f -- "${BASH_SOURCE[0]}")"
ruka3_root="$(cd -- "$(dirname -- "$ruka3_script")/.." && pwd)"
if [[ ! -f "$ruka3_root/build/ros2-install/local_setup.bash" ]]; then
  echo "Build RUKA3 ROS packages first; see $ruka3_root/ros2/README.md" >&2
  exit 1
fi
# Keep ROS networking and WSLg settings from the caller, isolate old workspaces.
exec env -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u CMAKE_PREFIX_PATH \
  -u LD_LIBRARY_PATH -u PYTHONPATH \
  bash --noprofile --norc -c '
    source "$1/ros2/wsl_setup.bash"
    shift
    exec ros2 launch ruka3_moveit_config without_cont.launch.py "$@"
  ' ruka3-moveit "$ruka3_root" "$@"
