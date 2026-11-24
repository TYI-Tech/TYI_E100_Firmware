#!/usr/bin/env python3
"""Delay just the LiDAR process until the kernel selects its configured source IP."""
import json
import os
from pathlib import Path
import socket
import sys
import time

root = Path(os.environ.get("UAV_CONFIG_DIR", "/opt/uav/configs"))
config = json.loads((root / "livox/MID360_config.json").read_text())
device = config["lidar_configs"][0]["ip"]
network = config.get("Mid360s") or config["MID360"]
source = network["host_net_info"]["cmd_data_ip"]
settle = float(os.environ.get("LIVOX_NET_STABILIZE_SEC", "3"))
stable_since = None
last_source = None
while True:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as route:
        try:
            # UDP connect performs a route lookup without transmitting a packet.
            route.connect((device, network["lidar_net_info"]["cmd_data_port"]))
            current = route.getsockname()[0]
        except OSError:
            current = "unreachable"
    now = time.monotonic()
    if current != last_source:
        print("Livox route to {} selects {} (expected {})".format(device, current, source), flush=True)
        last_source = current
    if current == source:
        if stable_since is None:
            stable_since = now
        if now - stable_since >= settle:
            print("Livox source route ready; starting driver", flush=True)
            os.execvp(sys.argv[1], sys.argv[1:])
    else:
        stable_since = None
    time.sleep(.5)
