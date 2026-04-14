#!/usr/bin/env bash
set -euo pipefail

export ROS_MASTER_URI="${ROS_MASTER_URI:-http://127.0.0.1:${ROS_MASTER_PORT:-11311}}"
export ROS_HOSTNAME="${ROS_HOSTNAME:-127.0.0.1}"
source /opt/ros/noetic/setup.bash

export DEBIAN_FRONTEND=noninteractive

apt-get update

# Normalize vendor scripts in case the workspace was prepared on Windows.
find /opt/uav/base_stack -type f \( -name '*.sh' -o -name '*.bash' -o -name '*.py' \) -exec sed -i 's/\r$//' {} +

# livox_ros_driver2 keeps edition-specific manifests upstream. Normalize the ROS1
# manifest so catkin/rosdep can discover the package during a clean build.
LIVOX_DRIVER_DIR="/opt/uav/base_stack/workspace/src/livox_ros_driver2"
if [[ -f "${LIVOX_DRIVER_DIR}/package_ROS1.xml" ]]; then
  cp "${LIVOX_DRIVER_DIR}/package_ROS1.xml" "${LIVOX_DRIVER_DIR}/package.xml"
fi

rosdep init 2>/dev/null || true
rosdep update --rosdistro noetic || echo "rosdep update failed, continuing with existing cache"

# Install heavy PCL ROS dependencies without recommended desktop packages.
apt-get install -y --no-install-recommends ros-noetic-pcl-ros

for attempt in 1 2 3; do
  if rosdep install \
    --from-paths /opt/uav/base_stack/workspace/src \
    --ignore-src \
    --rosdistro noetic \
    -r -y; then
    break
  fi

  if [[ "${attempt}" -eq 3 ]]; then
    echo "rosdep install failed after ${attempt} attempts" >&2
    exit 1
  fi

  echo "rosdep install attempt ${attempt} failed, retrying after apt-get update" >&2
  apt-get update
  sleep 5
done

bash /opt/uav/base_stack/workspace/src/mavros/mavros/scripts/install_geographiclib_datasets.sh

if command -v geographiclib-get-geoids >/dev/null 2>&1; then
  geographiclib-get-geoids egm96-5
fi

if [[ ! -r /usr/share/GeographicLib/geoids/egm96-5.pgm ]] && [[ ! -r /usr/local/share/GeographicLib/geoids/egm96-5.pgm ]]; then
  echo "Required GeographicLib geoid dataset egm96-5.pgm is missing" >&2
  exit 1
fi

cmake -S /opt/uav/base_stack/third_party/Livox-SDK2 \
      -B /opt/uav/base_stack/third_party/Livox-SDK2/build \
      -DCMAKE_BUILD_TYPE=Release
cmake --build /opt/uav/base_stack/third_party/Livox-SDK2/build --parallel "$(nproc)"
cmake --install /opt/uav/base_stack/third_party/Livox-SDK2/build

ln -sf /opt/ros/noetic/share/catkin/cmake/toplevel.cmake /opt/uav/base_stack/workspace/src/CMakeLists.txt

cd /opt/uav/base_stack/workspace
catkin_make_isolated \
  --install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS1

rm -rf /var/lib/apt/lists/*
