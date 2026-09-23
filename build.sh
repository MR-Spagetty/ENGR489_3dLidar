#!/bin/bash
# Build script for ENGR489 robot platform
# Normal build (not --symlink-install) to enable CMake install(CODE) for libexec setup

set -e

WORKSPACE="${1:-.}"
if [ $# -gt 0 ]; then
  shift
fi
cd "$WORKSPACE"

echo "🔨 Building ENGR489_3dLidar platform..."
echo "   Workspace: $WORKSPACE"

REQUIRED_PACKAGES=(
  eeen489_robot
  sllidar_ros2
  joy
  teleop_twist_joy
  twist_mux
)

echo "   Packages: ${REQUIRED_PACKAGES[*]}"

# Run colcon build
source ~/ros2_env/bin/activate
source ~/ros2_jazzy/install/setup.bash

colcon build \
  --packages-up-to "${REQUIRED_PACKAGES[@]}" \
  --parallel-workers 4 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  "$@"

echo ""
echo "✅ Build complete!"
echo "   Libexec symlinks created by CMake install(CODE)"
echo "   Use: source install/setup.bash"
echo "   Then: ros2 launch eeen489_robot EEEN325_Robot_Platform.launch.py"

