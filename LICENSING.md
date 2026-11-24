# Component licensing

The repository contains components with separate license declarations. The firmware has no repository-wide open-source license.

| Component | License information |
| --- | --- |
| TYI bringup | The [ROS package manifest](workspace/src/uav_base_bringup/package.xml) declares Proprietary. Contact the rights holder for reuse permissions. |
| FAST-LIO2 | The [bundled license](workspace/src/fast_lio/LICENSE) and the [upstream project](https://github.com/hku-mars/FAST_LIO) identify GPL v2. The source manifest in the initial v0.1.3 checkout carried a BSD label. The corrected manifest follows the bundled license. Consult individual source notices for embedded dependencies. |
| fastlio_to_mavros | The [ROS package manifest](workspace/src/fastlio_to_mavros/package.xml) declares MIT. |
| Livox driver and SDK | See the [driver license](workspace/src/livox_ros_driver2/LICENSE.txt) and [SDK license](third_party/Livox-SDK2-mid360s/LICENSE.txt). |
| MAVLink and MAVROS | See [MAVLink copying terms](workspace/src/mavlink/COPYING), [pymavlink copying terms](workspace/src/mavlink/pymavlink/COPYING) and the [MAVROS licensing guide](workspace/src/mavros/LICENSE.md). |
| GeographicLib grid | The [runtime notice](docker/runtime/THIRD_PARTY_NOTICES.md) identifies the bundled dataset. |
