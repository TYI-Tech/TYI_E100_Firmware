# Release Notes

## 0.1.2

This release adds dual deployment modes on top of `0.1.1`, and also publishes the aligned prebuilt image tag `0.1.2`.

Main updates:

- switches the default deployment path to the published ACR image
- keeps local source build support through `bash ./scripts/deploy.sh --build`
- adds `docker-compose.build.yml` as the Compose override for source builds
- updates deployment and operation scripts to support both pull mode and build mode
- expands the bilingual documentation for ACR login, prebuilt image usage, and source-build fallback
- verifies on `uav-nx` that the default image-pull path can authenticate to ACR and pull the published image successfully
- publishes aligned ACR tags `0.1.2`, `latest`, and `stable`

Operational conclusion:

- users can now prefer the prebuilt image for faster deployment
- users without registry access can still deploy from source with `--build`
- the default published runtime image is now `0.1.2`, aligned with the repository workflow version

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
