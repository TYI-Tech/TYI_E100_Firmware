"""Read the flight-core timebase shared by Livox and ROS."""
import os
from pathlib import Path
import time

SHARED_TIMEBASE_MODES = {"monotonic_raw_internal", "monotonic_raw_epoch"}
DEFAULT_TIMEBASE_FILE = "/tmp/tyi_timebase.env"


def monotonic_raw_now_ns():
    clock_id = getattr(time, "CLOCK_MONOTONIC_RAW", None)
    if clock_id is not None:
        return time.clock_gettime_ns(clock_id)
    return time.monotonic_ns()


def load_timebase_offset_ns():
    path = Path(os.environ.get("TYI_TIMEBASE_FILE", DEFAULT_TIMEBASE_FILE))
    values = {}
    for line in path.read_text(encoding="ascii").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key.strip()] = value.strip()

    mode = values.get("TYI_TIMEBASE_MODE")
    expected_mode = os.environ.get("TYI_TIMEBASE_MODE")
    if mode not in SHARED_TIMEBASE_MODES or (expected_mode and mode != expected_mode):
        raise ValueError(f"unexpected flight timebase mode {mode!r} in {path}")

    internal_anchor = int(values.get("INTERNAL_ANCHOR_NS") or values["UNIX_ANCHOR_NS"])
    raw_anchor = int(values["MONOTONIC_RAW_ANCHOR_NS"])
    if internal_anchor <= 0 or raw_anchor <= 0:
        raise ValueError(f"invalid flight timebase anchor in {path}")
    return internal_anchor - raw_anchor
