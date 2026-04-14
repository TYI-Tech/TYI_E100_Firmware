# Operations

These commands are the main firmware operation entrypoints.

## Start or redeploy

```bash
bash ./scripts/deploy.sh
```

## Rebuild only

```bash
bash ./scripts/build.sh
```

`build.sh` rebuilds the firmware image from the current source tree.
The current release has been validated with a clean source build and includes retry handling for base dependency installation.

## Start or stop the container

```bash
bash ./scripts/up.sh
bash ./scripts/down.sh
```

## Inspect state

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

`enter.sh` opens a shell inside the `base-stack` container.
The runtime user inside the container is `uav`, which avoids routine root execution during operation.
