# Quick Start

This repository is intended for product firmware deployment on Ubuntu 20.04 with Docker and Docker Compose available.
The only file that normally needs editing is `machine.env`.
The default recommendation is to pull the prebuilt image first, and only switch to source build when customization is required.

## First deployment

```bash
git clone git@github.com:TYI-Tech/TYI_E100_Firmware.git
cd TYI_E100_Firmware
bash ./scripts/check_host.sh
docker login --username=hyzrichard crpi-zpvbhgsm3t97idht.cn-hangzhou.personal.cr.aliyuncs.com
vim machine.env
bash ./scripts/deploy.sh
```

`deploy.sh` performs three actions in order:

- applies machine-specific UART, MID360, and host NIC settings
- pulls the prebuilt Docker image by default
- starts the runtime container in the background

`check_host.sh` verifies that Docker, Docker Compose, and the required configuration files are present before deployment.

Expected result after `deploy.sh`:

- the prebuilt runtime image is pulled locally
- the `base-stack` service is started in the background
- the runtime can be inspected through `status.sh`, `logs.sh`, and `enter.sh`

To switch to a local source build:

```bash
bash ./scripts/deploy.sh --build
```

This mode applies `docker-compose.build.yml` as an additional Compose override.
It is also the recommended fallback when the target machine cannot access the image registry.

Additional notes:

- the current release has been verified with a clean on-board source build
- the default runtime path uses the prebuilt image from ACR for faster deployment
- the required GeographicLib geoid file is bundled with the repository, so this resource is not fetched from SourceForge during build
- the image bootstrap path retries base `apt` installation when transient network failures occur

## After deployment

Use the following commands for routine operation:

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

If Docker on the target machine requires elevated privileges, the scripts will fall back to `sudo docker compose`.
