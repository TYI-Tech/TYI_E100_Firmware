#!/usr/bin/env python3
import argparse
import math
import os
import threading
import time

import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from internal_timebase import load_timebase_offset_ns, monotonic_raw_now_ns

DEFAULT_PUBLISH_HZ = 200.0
CLOCK_FUTURE_TOLERANCE_NS = 5_000_000


def make_ros_time(ns: int) -> rospy.Time:
    secs, nsecs = divmod(ns, 1_000_000_000)
    return rospy.Time(secs=secs, nsecs=nsecs)


def publish_clock(rate):
    offset_ns = load_timebase_offset_ns()
    rospy.init_node("shared_timebase_clock")
    publisher = rospy.Publisher("/clock", Clock, queue_size=1, tcp_nodelay=True)
    period_ns = round(1e9 / rate)
    deadline_ns = time.monotonic_ns()
    rospy.loginfo("shared_timebase_clock ready at %.1f Hz", rate)
    while not rospy.is_shutdown():
        publisher.publish(Clock(clock=make_ros_time(monotonic_raw_now_ns() + offset_ns)))
        deadline_ns += period_ns
        remaining_ns = deadline_ns - time.monotonic_ns()
        if remaining_ns > 0:
            time.sleep(remaining_ns / 1e9)
        else:
            deadline_ns = time.monotonic_ns()


def check_streams(rate, timeout, with_odometry):
    offset_ns = load_timebase_offset_ns()
    if rospy.get_param("/use_sim_time", False) is not True:
        raise RuntimeError("/use_sim_time must be true for the shared timebase")
    rospy.init_node("flight_timebase_check", anonymous=True, disable_rosout=True)
    streams = {"/clock": (Clock, max(0.1, 3.0 / rate))}
    if with_odometry:
        ekf = "/high_rate_odom_ekf"
        lio_topic = rospy.get_param(f"{ekf}/odom_topic", "/tyi/e100/fastlio2/odom")
        streams[lio_topic] = (Odometry, rospy.get_param(f"{ekf}/max_lio_age_sec", 3.0))
        if os.environ.get("ENABLE_HIGH_RATE_ODOM_EKF", "true") == "true":
            topic = rospy.get_param(f"{ekf}/output_topic", "/tyi/e100/ekf_odom")
            streams[topic] = (Odometry, rospy.get_param(f"{ekf}/max_imu_age_sec", 0.25))

    lock = threading.Lock()
    previous = {}
    advanced = set()
    error = []

    def receive(msg, topic):
        stamp = msg.clock if topic == "/clock" else msg.header.stamp
        stamp_ns = stamp.to_nsec()
        with lock:
            prev_ns = previous.get(topic, 0)
            if stamp_ns < prev_ns:
                error.append(f"{topic}: timestamp moved backwards")
            if prev_ns and stamp_ns > prev_ns:
                advanced.add(topic)
            previous[topic] = stamp_ns

    subscribers = [
        rospy.Subscriber(topic, msg_type, receive, callback_args=topic, queue_size=1, tcp_nodelay=True)
        for topic, (msg_type, _) in streams.items()
    ]
    deadline = time.monotonic() + timeout
    try:
        while not rospy.is_shutdown():
            with lock:
                if error:
                    raise RuntimeError(error[0])
                now_ns = monotonic_raw_now_ns() + offset_ns
                problems = []
                for topic, (_, max_age_sec) in streams.items():
                    age_ns = now_ns - previous.get(topic, 0)
                    if topic not in advanced:
                        problems.append(f"{topic}: no advancing timestamps")
                    elif not -CLOCK_FUTURE_TOLERANCE_NS <= age_ns <= max_age_sec * 1e9:
                        problems.append(f"{topic}: timestamp age {age_ns / 1e6:.1f} ms outside bounds")
            if not problems:
                print("timebase healthy: " + ", ".join(streams))
                return
            if time.monotonic() >= deadline:
                raise RuntimeError("; ".join(problems))
            time.sleep(0.02)
        raise RuntimeError("ROS shut down before validating the timebase")
    finally:
        for subscriber in subscribers:
            subscriber.unregister()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--with-odometry", action="store_true")
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args()

    try:
        rate = float(os.environ.get("SIM_TIME_PUBLISH_HZ", DEFAULT_PUBLISH_HZ))
        if not math.isfinite(rate) or not 1.0 <= rate <= 1e9:
            raise ValueError("SIM_TIME_PUBLISH_HZ must be a finite rate of at least 1 Hz")
        if args.check:
            if not math.isfinite(args.timeout) or args.timeout <= 0:
                raise ValueError("--timeout must be positive and finite")
            check_streams(rate, args.timeout, args.with_odometry)
        else:
            publish_clock(rate)
    except (OSError, KeyError, ValueError, RuntimeError, rospy.ROSException) as exc:
        parser.exit(1, f"flight timebase: {exc}\n")


if __name__ == "__main__":
    main()
