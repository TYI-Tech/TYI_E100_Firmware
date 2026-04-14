# Quick Start

This package is intended for customer deployment on Ubuntu 20.04 with Docker and Docker Compose available.
The only file that normally needs editing is `machine.env`.

## First deployment

```bash
cd firmware/base_stack_noetic_customer
bash ./scripts/check_host.sh
vim machine.env
bash ./scripts/deploy.sh
```

`deploy.sh` performs three actions in order:

- applies machine-specific UART, MID360, and host NIC settings
- builds the Docker image from the local source tree
- starts the runtime container in the background

`check_host.sh` verifies that Docker, Docker Compose, and the required configuration files are present before deployment.

## After deployment

Use the following commands for routine operation:

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

If Docker on the target machine requires elevated privileges, the scripts will fall back to `sudo docker compose`.
