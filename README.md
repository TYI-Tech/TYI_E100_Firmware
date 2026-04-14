# TYI UAV Firmware

This repository is the customer-facing firmware package for the UAV-NX ROS1 base stack.
It keeps the source code visible and lets customers build and run the runtime directly with `docker compose`.

Chinese version: [README_CN.md](README_CN.md)

Current version: [VERSION](VERSION)

## Included modules

- `livox_ros_driver2`
- `dlio`
- `fastlio_to_mavros`
- `mavros`
- `mavlink`
- `uav_base_bringup`

## Customer workflow

1. Edit [machine.env](machine.env) for the target airframe.
2. Run `bash ./scripts/deploy.sh`.
3. Confirm the container state with `bash ./scripts/status.sh`.
4. Use `bash ./scripts/logs.sh` or `bash ./scripts/enter.sh` for routine operation checks.

The customer package keeps only deployment and operation entrypoints.
Internal smoke tests and development-only debugging scripts are intentionally not included here.

## Quick links

- [English quick start](docs/en_US/quick_start.md)
- [Chinese quick start](docs/zh_CN/快速上手.md)
- [English docs index](docs/en_US/README.md)
- [Chinese docs index](docs/zh_CN/README.md)
- [Changelog](CHANGELOG.md)

## Optional bridge package

If the project later needs control bridge capabilities, install `TYI_Plugin_Ctl` separately through the apt package:

```bash
sudo apt install tyi-plugin-ctl
```
