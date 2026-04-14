# Release Notes

## 0.1.1

This release closes the clean source-build workflow for the product firmware repository and has been validated on the `uav-nx` production board.

Main updates:

- supports `docker compose build --no-cache base-stack` directly from the checked-out source tree
- normalizes the ROS1 manifest for `livox_ros_driver2` during image build so fresh builds can discover the workspace package reliably
- bundles the required GeographicLib `egm96-5` geoid dataset with the repository instead of downloading it separately during build
- adds retry handling for the base `apt` dependency bootstrap path to reduce transient DNS or mirror failures
- validated runtime chain:
  `livox_lidar_publisher2`, `robot/dlio_odom`, `fastlio_to_mavros`, `mavros`
- validated key topics:
  `/robot/dlio/odom_node/odom`, `/mavros/vision_pose/pose`

Operational conclusion:

- users can now clone this repository and build the runtime directly with Docker Compose
- machine-specific differences should remain in `machine.env`
- control bridge capability continues to be delivered separately through `TYI_Plugin_Ctl`
