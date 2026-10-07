#!/bin/bash
# Build script for ENGR489 robot platform
# Normal build (not --symlink-install) to enable CMake install(CODE) for libexec setup

set -e

WORKSPACE="${1:-.}"
cd "$WORKSPACE"

echo "🔨 Building nav2 stuff..."
echo "   Workspace: $WORKSPACE"

# Run colcon build
source ~/ros2_env/bin/activate
source ~/ros2_jazzy/install/setup.bash

colcon build \
  --parallel-workers 4 \
  --packages-skip cv_bridge image_geometry opencv_tests vision_opencv robot_localization nav2_mppi_controller nav2_waypoint_follower nav2_rviz_plugins navigation2 nav2_bringup nav2_system_tests \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS=-Wno-error=null-dereference \
  -DCV_BRIDGE_DISABLE_PYTHON=ON \
  "$@"

echo ""
echo "✅ Build complete!"
echo "   Libexec symlinks created by CMake install(CODE)"
echo "   Use: source install/setup.bash"
