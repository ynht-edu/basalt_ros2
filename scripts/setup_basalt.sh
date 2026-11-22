#!/usr/bin/env bash
# usage: ./setup_basalt.sh [workspace]   default ~/inform_ws
set -e

WS=$(realpath "${1:-$HOME/inform_ws}")
ROS_DISTRO=${ROS_DISTRO:-jazzy}
BASALT_DIR=$WS/src/thirdparty/basalt

step() { echo -e "\n\033[1;34m==> $1\033[0m"; }
fail() { echo -e "\033[1;31merror: $1\033[0m"; exit 1; }

if [ -n "$CONDA_PREFIX" ]; then
    fail "conda environment '$CONDA_DEFAULT_ENV' is active, run 'conda deactivate' first"
fi
[ -f /opt/ros/$ROS_DISTRO/setup.bash ] || fail "ROS 2 $ROS_DISTRO not found in /opt/ros"
source /opt/ros/$ROS_DISTRO/setup.bash
cd "$WS"

step "basalt sources"
if [ ! -f "$BASALT_DIR/src/vi_estimator/sqrt_keypoint_vio.cpp" ]; then
    git -C "$WS/src" submodule update --init thirdparty/basalt
fi
[ -f "$BASALT_DIR/src/vi_estimator/sqrt_keypoint_vio.cpp" ] || fail "basalt not found in $BASALT_DIR"
if [ ! -f "$BASALT_DIR/COLCON_IGNORE" ]; then
    touch "$BASALT_DIR/COLCON_IGNORE"
    echo "added COLCON_IGNORE, commit it in the basalt fork"
fi
echo "ok: $BASALT_DIR"

step "system dependencies (rosdep)"
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
    sudo rosdep init
fi
rosdep update --rosdistro "$ROS_DISTRO" > /dev/null
BASALT_ROS=$(colcon list --packages-select basalt_ros --paths-only)
[ -n "$BASALT_ROS" ] || fail "basalt_ros package not found in $WS/src"
rosdep install --from-paths "$BASALT_ROS" --ignore-src --rosdistro "$ROS_DISTRO" -y

step "check"
for header in tbb/tbb.h fmt/format.h nlohmann/json.hpp opencv4/opencv2/core.hpp eigen3/Eigen/Core; do
    [ -f /usr/include/$header ] || fail "missing /usr/include/$header"
done
echo "ok: all headers found"

MEM_GB=$(awk '/MemTotal/ { printf "%d", $2 / 1024 / 1024 }' /proc/meminfo)
JOBS=$(( MEM_GB / 3 ))
JOBS=$(( JOBS < 1 ? 1 : JOBS ))
JOBS=$(( JOBS > $(nproc) ? $(nproc) : JOBS ))

step "build basalt_ros with $JOBS jobs (${MEM_GB} GB ram, ~3 GB per job)"
rm -rf "$WS/build/basalt"
MAKEFLAGS=-j$JOBS colcon build --packages-select basalt_ros \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DBASALT_SOURCE_DIR="$BASALT_DIR"

step "done"
echo "source $WS/install/setup.bash"
echo "ros2 launch uav_bringup uav.launch.xml vio_method:=basalt"
