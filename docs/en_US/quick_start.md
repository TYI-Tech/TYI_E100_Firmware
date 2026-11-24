# Quick Start

Use a Linux ARM64 host with Docker Engine and Docker Compose capable of parsing the shipped Compose file. The default install requires Bash, Python 3, curl, zstd and sha256sum. The image runs Ubuntu 20.04 and ROS Noetic.

## First deployment

```bash
git clone --branch v0.1.3 https://github.com/TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
bash ./scripts/check_host.sh
```

Fill in the serial port, LiDAR serial and IP settings, ground station address and MAVLink system ID in `machine.env`. Use `ip -br -4 addr` to confirm the LiDAR interface has the configured host IP. For a dedicated LiDAR interface that needs configuration, see [Machine configuration](machine_configuration.md).

```bash
bash ./deploy.sh
bash ./validate_stack.sh
```

The default install verifies the SHA-256 of the Linux ARM64 archive from GitHub Releases. Use `bash ./deploy.sh --local` with an already loaded image or `bash ./deploy.sh --build` to build the checkout.

Acceptance prints `flight-core healthy`. If startup times out, run `bash ./status.sh` and `bash ./logs.sh` and inspect the LiDAR, odometry and clock diagnostics.
