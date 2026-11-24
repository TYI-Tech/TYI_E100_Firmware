# Host Check

Run the host preflight check before the first deployment:

```bash
bash ./scripts/check_host.sh
```

The script checks Docker, Compose and configuration templates, creates a missing `machine.env` from the example, and validates the Compose file. Verify the UART, LiDAR interface address and sensor streams on the target device.
