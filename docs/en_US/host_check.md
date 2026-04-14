# Host Check

Run the host preflight check before the first deployment:

```bash
bash ./scripts/check_host.sh
```

The script checks:

- `docker` command availability
- `docker compose` availability
- presence of `.env`
- presence of `machine.env`
- presence of `configs/livox/MID360_config.json`

It does not modify the machine.
