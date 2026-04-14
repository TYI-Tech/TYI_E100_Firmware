# Machine Configuration

This firmware package is designed so that each airframe only needs one editable machine file:

- `machine.env`

## Parameters most users need to change

- `FCU_DEVICE`
  PX4 to onboard computer UART device, for example `/dev/ttyTHS1`
- `FCU_BAUD`
  PX4 serial baud rate
- `MID360_BD_LIST`
  MID360 serial number used by the Livox driver
- `MID360_SN_SUFFIX`
  last two digits of the MID360 serial number, used to derive `192.168.1.(100 + suffix)`
- `MID360_LIDAR_IP`
  optional fixed override if the default IP derivation rule should not be used
- `HOST_ETH_IFACE`
  host Ethernet interface connected to MID360
- `HOST_ETH_IP`
  host IP used for MID360 communication, based on the deployment network plan, for example `192.168.1.50` or `192.168.1.5`

## What `configure_machine.sh` updates

When `bash ./scripts/deploy.sh` runs, it calls `scripts/configure_machine.sh` and updates:

- `.env`
  writes `FCU_URL` and `LIVOX_BD_LIST`
- `configs/livox/MID360_config.json`
  writes host IP, MID360 IP, and related UDP ports
- host Ethernet configuration
  applies the configured IP to the Ethernet interface connected to MID360

## Notes

- Keep the main firmware workflow focused on `machine.env`; avoid editing `.env` unless there is a deployment-specific need.
- `configure_machine.sh` rewrites `.env` and `MID360_config.json` from `machine.env`, so the same machine data does not need to be edited in multiple places.
- The LIO topic and bridge reference frame remain configurable through `configs/fastlio_to_mavros/bridge.yaml` if the stack later integrates `TYI_Plugin_Ctl`.
