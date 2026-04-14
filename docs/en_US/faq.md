# FAQ

## Which file should customers edit first?

`machine.env`

This file is the main customer-edit surface for UART, MID360 serial/IP, and host Ethernet settings.

## Do customers need to edit `.env`?

Usually no.

`.env` keeps deployment defaults and is updated automatically by `scripts/configure_machine.sh`.

## Can the LIO topic or reference frame be changed?

Yes.

Use `configs/fastlio_to_mavros/bridge.yaml` to change the bridge input topic or reference frame when downstream control integration requires it.

## Is `TYI_Plugin_Ctl` included in this repository?

No.

This repository only contains the firmware-side runtime stack.
`TYI_Plugin_Ctl` is distributed separately as an apt package.
