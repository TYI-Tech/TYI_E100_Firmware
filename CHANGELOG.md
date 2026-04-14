# Changelog

## 0.1.2

- Added dual deployment modes for the product repository: prebuilt image pull by default, local source build with `--build`
- Switched the default product image reference to the published ACR image `crpi-zpvbhgsm3t97idht.cn-hangzhou.personal.cr.aliyuncs.com/tyi-tech/tyi_e100:0.1.1`
- Added Compose override file `docker-compose.build.yml` for local source builds while keeping the default Compose path image-based
- Updated deployment and operation scripts to support both pull mode and build mode
- Expanded Chinese and English documentation to explain ACR login, prebuilt image usage, and the fallback source-build workflow
- Verified on `uav-nx` that the default image-pull path can authenticate to ACR and pull the published image successfully
- Published aligned ACR image tags `0.1.2`, `latest`, and `stable`

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
