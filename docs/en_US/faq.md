# FAQ

## Which file should be edited first?

`machine.env`

This file is the main editable surface for UART, MID360 serial/IP, and host Ethernet settings.

## Does `.env` need to be edited?

Usually no.

`.env` keeps deployment defaults and is updated automatically by `scripts/configure_machine.sh`.

## Does the build still depend on downloading GeographicLib data online?

Not by default.

The required `egm96-5` geoid file is now bundled with the repository so the build does not rely on an extra SourceForge download for this resource.

## Does the build still require network access?

Yes.

The build still needs Ubuntu and ROS package sources for system dependencies, but the base installation path now retries automatically to reduce transient DNS or mirror failures.

## Can the LIO topic or reference frame be changed?

Yes.

Use `configs/fastlio_to_mavros/bridge.yaml` to change the bridge input topic or reference frame when downstream control integration requires it.

## Is `TYI_Plugin_Ctl` included in this repository?

No.

This repository only contains the firmware-side runtime stack.
`TYI_Plugin_Ctl` is distributed separately as an apt package.
