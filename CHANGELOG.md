# Changelog

## 0.1.1

- Verified clean source build on the `uav-nx` production board with `docker compose build --no-cache base-stack`
- Normalized the ROS1 `livox_ros_driver2` manifest during image build so the workspace can be discovered reliably in fresh builds
- Bundled the required GeographicLib `egm96-5` geoid dataset into the firmware repository to remove the SourceForge runtime download dependency
- Added retry handling for base `apt` bootstrap during image build to reduce transient DNS or mirror failures
- Rebuilt and revalidated the runtime stack on board, including `livox_lidar_publisher2`, `robot/dlio_odom`, `fastlio_to_mavros`, `mavros`, `/robot/dlio/odom_node/odom`, and `/mavros/vision_pose/pose`

## 0.1.0

- Initial product firmware repository extracted from the production `uav-nx` base stack
- Source-visible Docker Compose bring-up for `livox_ros_driver2`, `dlio`, `fastlio_to_mavros`, `mavros`, and `mavlink`
- Single-machine configuration surface through `machine.env`
- Bilingual firmware documentation in `docs/en_US` and `docs/zh_CN`
- Removed internal smoke-test and development-only operation entrypoints from the product firmware package
