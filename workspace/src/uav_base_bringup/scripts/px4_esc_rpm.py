#!/usr/bin/env python3
"""Publish four ESC speeds together with their original PX4 batch timestamp."""

import struct

import rospy
from mavros_msgs.msg import Mavlink
from uav_base_bringup.msg import EscRpm


ESC_STATUS_ID = 291
# time_usec, rpm[4], voltage[4] and current[4] (skipped), then batch index.
ESC_STATUS = struct.Struct("<Q4i32xB")


def decode_esc_rpm(message):
    if message.msgid != ESC_STATUS_ID or message.framing_status != Mavlink.FRAMING_OK:
        return None
    payload = b"".join(word.to_bytes(8, "little") for word in message.payload64)
    if not 0 < message.len <= len(payload):
        return None
    # MAVLink 2 omits trailing zero bytes, including index 0.
    payload = payload[:message.len].ljust(ESC_STATUS.size, b"\0")
    timestamp, rpm1, rpm2, rpm3, rpm4, index = ESC_STATUS.unpack_from(payload)
    if index != 0:
        return None
    return EscRpm(px4_timestamp_us=timestamp, rpm=[rpm1, rpm2, rpm3, rpm4])


def main():
    rospy.init_node("px4_esc_rpm")
    target = (rospy.get_param("~target_system", 1), rospy.get_param("~target_component", 1))
    output = rospy.Publisher("/tyi/e100/esc_rpm", EscRpm, queue_size=100)

    def receive(message):
        if (message.sysid, message.compid) == target:
            sample = decode_esc_rpm(message)
            if sample is not None:
                output.publish(sample)

    subscriber = rospy.Subscriber("/mavlink/from", Mavlink, receive, queue_size=1000)
    rospy.spin()
    subscriber.unregister()


if __name__ == "__main__":
    main()
