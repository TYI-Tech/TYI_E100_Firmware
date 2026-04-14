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
- single-machine configuration entry through [machine.env](machine.env)
- Docker-based bring-up for `livox_ros_driver2`, `dlio`, `fastlio_to_mavros`, `mavros`, `mavlink`, and `uav_base_bringup`
- product firmware deployment and operation scripts

## Runtime Pipeline

`Livox MID360 -> DLIO -> fastlio_to_mavros -> MAVROS -> PX4`

## Quick Start

```bash
git clone git@github.com:TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
bash ./scripts/check_host.sh
vim machine.env
bash ./scripts/deploy.sh
```

For a local source build instead:

```bash
bash ./scripts/deploy.sh --build
```

To pull the prebuilt image manually:

```bash
docker pull crpi-zpvbhgsm3t97idht.cn-hangzhou.personal.cr.aliyuncs.com/tyi-tech/tyi_e100:0.1.2
```

Optional stable alias:

```bash
docker pull crpi-zpvbhgsm3t97idht.cn-hangzhou.personal.cr.aliyuncs.com/tyi-tech/tyi_e100:stable
```

After deployment:

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

## Shortest Path

For a first-time bring-up with the fewest decisions, use this order:

1. clone the repository and run `bash ./scripts/check_host.sh`
2. edit only [machine.env](machine.env)
3. run `bash ./scripts/deploy.sh`
4. confirm the runtime with `bash ./scripts/status.sh` and `bash ./scripts/logs.sh`

Notes:

- `deploy.sh` pulls the prebuilt ACR image and starts it by default
- `deploy.sh --build` switches to local source-build mode
- the build now includes the required GeographicLib geoid locally and retries the base `apt` bootstrap path
- machine-specific differences are expected to stay within `machine.env`

## Common Configuration Entry Points

- [machine.env](machine.env)
  airframe UART, MID360 serial/IP, and host NIC settings
- [configs/fastlio_to_mavros/bridge.yaml](configs/fastlio_to_mavros/bridge.yaml)
  bridge topic and frame settings when downstream control integration is needed
- [configs/dlio](configs/dlio)
  DLIO runtime parameters
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
