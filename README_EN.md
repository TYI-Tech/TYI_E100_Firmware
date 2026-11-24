<p align="center">
  <img src="assets/logo.png" alt="TYI Innovation" width="260">
</p>

# TYI E100 Firmware

ROS1 product firmware repository for `TYI E100`.
This repository keeps the runtime source visible and provides the build and runtime entrypoints through `docker compose`.
The current release supports both:

- running from a prebuilt image
- building and running directly from the checked-out source tree

Chinese homepage: [README.md](README.md)

Current version: [VERSION](VERSION)

## What This Repository Delivers

- source-visible deployment package for Ubuntu 20.04 + ROS1 Noetic
- single-machine configuration entry through [machine.env.example](machine.env.example)
- Docker-based bring-up for `livox_ros_driver2`, `fast_lio`, `fastlio_to_mavros`, `mavros`, `mavlink`, and `uav_base_bringup`
- product firmware deployment and operation scripts

## Runtime Pipeline

`Livox MID360 -> FAST-LIO2 -> high_rate_odom_ekf -> fastlio_to_mavros -> MAVROS -> PX4`

## Quick Start

```bash
git clone --branch v0.1.3 https://github.com/TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
bash ./scripts/check_host.sh
```

Fill in the serial port, LiDAR serial and IP settings, ground station address and MAVLink system ID in `machine.env`. Use `ip -br -4 addr` to confirm the LiDAR interface has the configured host IP.

```bash
bash ./deploy.sh
bash ./validate_stack.sh
```

The default install verifies the SHA-256 of the Linux ARM64 archive from GitHub Releases. Use `bash ./deploy.sh --local` with an already loaded image or `bash ./deploy.sh --build` to build the checkout.

## Common Configuration Entry Points

- [machine.env.example](machine.env.example)
  airframe UART, MID360 serial/IP, and host NIC settings
- [configs/fastlio_to_mavros/bridge.yaml](configs/fastlio_to_mavros/bridge.yaml)
  bridge topic and frame settings when downstream control integration is needed
- [configs/fastlio2](configs/fastlio2)
  FAST-LIO2 runtime parameters
- [configs/mavros](configs/mavros)
  MAVROS plugin and FCU parameters

## Start Here

- [Chinese quick start](docs/zh_CN/%E5%BF%AB%E9%80%9F%E4%B8%8A%E6%89%8B.md)
- [Chinese documentation](docs/zh_CN/README.md)
- [Chinese release notes](docs/zh_CN/%E7%89%88%E6%9C%AC%E8%AF%B4%E6%98%8E.md)
- [Documentation index](docs/README.md)
- [English quick start](docs/en_US/quick_start.md)
- [English documentation](docs/en_US/README.md)
- [English release notes](docs/en_US/release_notes.md)
- [Changelog](CHANGELOG.md)

## Repository Layout

- `configs/`
  runtime configuration mounted into the container
- `docker/`
  Docker build and runtime entrypoint files
- `scripts/`
  firmware operation entrypoints
- `third_party/`
  vendored third-party build dependencies
- `workspace/src/`
  ROS source packages used by the runtime

## Optional Bridge Package

If the project later needs control bridge capabilities, install `TYI_Plugin_Ctl` separately:

```bash
sudo apt install tyi-plugin-ctl
```

## Scope

This repository is intended for product firmware release, deployment, and field operation.
Internal smoke tests and development-only debugging entrypoints are intentionally excluded.

See the [Licensing](LICENSING.md) index.
