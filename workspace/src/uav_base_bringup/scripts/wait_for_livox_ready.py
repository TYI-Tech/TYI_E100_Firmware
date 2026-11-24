#!/usr/bin/env python3
"""Wait for simultaneous, sustained Livox IMU and point cloud input."""
import math
import os
import signal
import struct
import sys
import threading
import time
from collections import deque

import rospy


def setting(name, default, *, allow_zero=False):
    value = float(os.environ.get(name, default))
    if not math.isfinite(value) or value < 0 or (value == 0 and not allow_zero):
        raise ValueError(f"{name} must be finite and {'nonnegative' if allow_zero else 'positive'}")
    return value


def wait_for_livox():
    timeout = setting("LIVOX_TOPIC_WAIT_TIMEOUT_SEC", 45)
    settle = setting("LIVOX_TOPIC_STABILIZE_SEC", 6, allow_zero=True)
    window = setting("LIVOX_TOPIC_RATE_SAMPLE_SEC", 5)
    minimum = {
        "/livox/imu": setting("LIVOX_IMU_MIN_HZ", 120),
        "/livox/lidar": setting("LIVOX_LIDAR_MIN_HZ", 8),
    }
    max_age = {topic: max(0.25, 4 / rate) for topic, rate in minimum.items()}
    history = {topic: deque() for topic in minimum}
    lock = threading.Lock()

    def receive(message, topic):
        arrived = time.monotonic()
        try:
            # Both supported Livox messages start with std_msgs/Header.
            _, seconds, nanos = struct.unpack_from("<III", message._buff)
            stamp = seconds * 1_000_000_000 + nanos
        except struct.error:
            stamp = 0
        with lock:
            samples = history[topic]
            if samples and (stamp <= samples[-1][1] or arrived - samples[-1][0] > max_age[topic]):
                samples.clear()
            if stamp > 0:
                samples.append((arrived, stamp))
                while len(samples) > 1 and samples[1][0] <= arrived - window:
                    samples.popleft()

    # Bound ROS registration as well as the data wait if the master disappears.
    signal.alarm(math.ceil(timeout + settle + 2))
    try:
        rospy.init_node("e100_livox_startup_check", anonymous=True, disable_rosout=True)
        subscribers = [
            rospy.Subscriber(topic, rospy.AnyMsg, receive, callback_args=topic,
                             queue_size=10, buff_size=4 * 1024 * 1024, tcp_nodelay=True)
            for topic in minimum
        ]
        deadline = time.monotonic() + timeout
        stable_since = None
        next_report = 0
        try:
            while not rospy.is_shutdown():
                with lock:
                    now = time.monotonic()
                    ros_now = rospy.Time.now().to_nsec()
                    rates = {}
                    ready = True
                    for topic, samples in history.items():
                        span = samples[-1][0] - samples[0][0] if samples else 0
                        rate = (len(samples) - 1) / span if span > 0 else 0
                        rates[topic] = rate
                        if not samples or (
                            span < window
                            or rate < minimum[topic]
                            or now - samples[-1][0] > max_age[topic]
                            or not -0.1 <= (ros_now - samples[-1][1]) / 1e9 <= max_age[topic]
                        ):
                            ready = False

                if ready:
                    if stable_since is None:
                        stable_since = now
                    if now - stable_since >= settle:
                        report = ", ".join(f"{topic} {rate:.1f} Hz" for topic, rate in rates.items())
                        print(f"Livox inputs ready at uptime {now:.3f}s: {report}", file=sys.stderr)
                        return 0
                else:
                    stable_since = None
                    if now >= deadline:
                        break
                if now >= next_report:
                    report = ", ".join(f"{topic} {rate:.1f}/{minimum[topic]:g} Hz"
                                       for topic, rate in rates.items())
                    print(f"Waiting for fresh Livox inputs: {report}", file=sys.stderr)
                    next_report = now + 5
                time.sleep(0.05)
        finally:
            for subscriber in subscribers:
                subscriber.unregister()
    finally:
        signal.alarm(0)

    print("Timed out waiting for sustained Livox IMU and point cloud input", file=sys.stderr)
    return 1


if __name__ == "__main__":
    try:
        sys.exit(wait_for_livox())
    except (ValueError, rospy.ROSException) as error:
        sys.exit(str(error))
