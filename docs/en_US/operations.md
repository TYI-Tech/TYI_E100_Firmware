# Operations

These commands are the main firmware operation entrypoints.

## Start or redeploy

```bash
bash ./scripts/deploy.sh
bash ./scripts/deploy.sh --build
```

- `deploy.sh`
  downloads and verifies the release image and starts it
- `deploy.sh --build`
  builds from the local source tree and then starts the runtime

## Rebuild only

```bash
bash ./scripts/build.sh
```

`build.sh` rebuilds the firmware image from the current source tree.
It uses the `docker-compose.build.yml` override for local source builds.

## Start or stop the container

```bash
bash ./scripts/up.sh
bash ./scripts/up.sh --build
bash ./scripts/down.sh
```

## Inspect state

```bash
bash ./scripts/status.sh
bash ./scripts/logs.sh
bash ./scripts/enter.sh
```

`enter.sh` opens a shell inside the `flight-core` container.
The runtime user inside the container is `uav`, which avoids routine root execution during operation.
