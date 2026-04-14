# Operations

These commands are the main customer-facing operation entrypoints.

## Start or redeploy

```bash
bash ./scripts/deploy.sh
```

## Rebuild only

```bash
bash ./scripts/build.sh
```

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
