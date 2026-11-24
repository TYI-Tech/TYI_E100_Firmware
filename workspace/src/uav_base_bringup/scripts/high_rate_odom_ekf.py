#!/usr/bin/env python3
import math
import os
import threading
import time
from collections import deque

import numpy as np
import rospy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool
import tf2_ros

from internal_timebase import SHARED_TIMEBASE_MODES, load_timebase_offset_ns, monotonic_raw_now_ns

FUTURE_TOLERANCE_NS = 5_000_000


def skew(v):
    return np.array(
        [
            [0.0, -v[2], v[1]],
            [v[2], 0.0, -v[0]],
            [-v[1], v[0], 0.0],
        ],
        dtype=float,
    )


def quat_normalize(q):
    n = np.linalg.norm(q)
    if n <= 1.0e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array(
        [
            aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
        ],
        dtype=float,
    )


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)


def quat_to_rot(q):
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


def delta_quat(theta):
    angle = np.linalg.norm(theta)
    if angle < 1.0e-8:
        return quat_normalize(np.array([1.0, 0.5 * theta[0], 0.5 * theta[1], 0.5 * theta[2]], dtype=float))
    axis = theta / angle
    half = 0.5 * angle
    return np.array([math.cos(half), *(math.sin(half) * axis)], dtype=float)


def quat_log(q):
    q = quat_normalize(q)
    if q[0] < 0.0:
        q = -q
    v = q[1:4]
    v_norm = np.linalg.norm(v)
    if v_norm < 1.0e-8:
        return 2.0 * v
    return 2.0 * math.atan2(v_norm, q[0]) * v / v_norm


def msg_quat_to_array(q):
    return quat_normalize(np.array([q.w, q.x, q.y, q.z], dtype=float))


class HighRateOdomEkf:
    def __init__(self):
        self.imu_topic = rospy.get_param("~imu_topic", "/livox/imu")
        self.odom_topic = rospy.get_param("~odom_topic", "/tyi/e100/fastlio2/odom")
        self.output_topic = rospy.get_param("~output_topic", "/tyi/e100/ekf_odom")
        self.lio_input_health_topic = rospy.get_param("~lio_input_health_topic", "")
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))
        self.output_child_frame_id = rospy.get_param("~output_child_frame_id", "")
        self.output_frame_id = rospy.get_param("~output_frame_id", "")
        self.buffer_sec = float(rospy.get_param("~buffer_sec", 3.0))
        self.max_replay_sec = float(rospy.get_param("~max_replay_sec", 2.0))
        self.max_dt = float(rospy.get_param("~max_imu_dt_sec", 0.05))
        self.max_lio_age_sec = float(rospy.get_param("~max_lio_age_sec", 2.0))
        self.max_imu_age_sec = float(rospy.get_param("~max_imu_age_sec", 0.25))
        if not all(math.isfinite(age) and age > 0 for age in (self.max_lio_age_sec, self.max_imu_age_sec)):
            raise ValueError("maximum LIO and IMU ages must be positive and finite")
        self.max_lio_age_ns = round(self.max_lio_age_sec * 1e9)
        self.max_imu_age_ns = round(self.max_imu_age_sec * 1e9)
        self.timebase_offset_ns = None
        if os.environ.get("TYI_TIMEBASE_MODE") in SHARED_TIMEBASE_MODES:
            self.timebase_offset_ns = load_timebase_offset_ns()
        self.imu_queue_size = int(rospy.get_param("~imu_queue_size", 80))
        self.odom_queue_size = int(rospy.get_param("~odom_queue_size", 10))
        self.publish_queue_size = int(rospy.get_param("~publish_queue_size", 100))
        self.publish_rate_hz = float(rospy.get_param("~publish_rate_hz", 0.0))
        self.reject_position_threshold_m = float(rospy.get_param("~reject_position_threshold_m", 2.0))
        self.use_imu_accel_for_position = bool(rospy.get_param("~use_imu_accel_for_position", False))
        self.odom_velocity_gain = float(rospy.get_param("~odom_velocity_gain", 0.85))
        self.velocity_random_walk = float(rospy.get_param("~velocity_random_walk", 0.20))
        self.gravity = np.array(rospy.get_param("~gravity", [0.0, 0.0, -9.80665]), dtype=float)

        gyro_noise = float(rospy.get_param("~gyro_noise", 0.05))
        accel_noise = float(rospy.get_param("~accel_noise", 0.35))
        gyro_bias_rw = float(rospy.get_param("~gyro_bias_random_walk", 0.0005))
        accel_bias_rw = float(rospy.get_param("~accel_bias_random_walk", 0.005))
        pos_cov = float(rospy.get_param("~position_cov", 0.02))
        rot_cov = float(rospy.get_param("~orientation_cov", 0.02))

        self.process_noise = np.diag(
            [
                accel_noise * accel_noise,
                accel_noise * accel_noise,
                accel_noise * accel_noise,
                gyro_noise * gyro_noise,
                gyro_noise * gyro_noise,
                gyro_noise * gyro_noise,
                gyro_bias_rw * gyro_bias_rw,
                gyro_bias_rw * gyro_bias_rw,
                gyro_bias_rw * gyro_bias_rw,
                accel_bias_rw * accel_bias_rw,
                accel_bias_rw * accel_bias_rw,
                accel_bias_rw * accel_bias_rw,
            ]
        )
        self.R_meas = np.diag([pos_cov, pos_cov, pos_cov, rot_cov, rot_cov, rot_cov])

        self.lock = threading.RLock()
        self.initialized = False
        self.p = np.zeros(3)
        self.v = np.zeros(3)
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
        self.bg = np.zeros(3)
        self.ba = np.zeros(3)
        self.P = np.eye(15) * 0.01
        self.P[3:6, 3:6] *= 10.0
        self.last_imu_stamp = None
        self.last_imu_received_ns = None
        self.frame_id = "robot/odom"
        self.child_frame_id = "base_link"
        self.latest_gyro = np.zeros(3)
        self.buffer = deque()
        self.last_odom_stamp = None
        self.last_odom_received_ns = None
        self.last_odom_position = None
        self.last_published_stamp = None
        self.lio_stale = False
        self.lio_frontend_ok = not self.lio_input_health_topic

        self.pub = rospy.Publisher(self.output_topic, Odometry, queue_size=self.publish_queue_size, tcp_nodelay=True)
        self.tf_pub = tf2_ros.TransformBroadcaster() if self.publish_tf else None
        self.publish_thread = None
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.on_imu, queue_size=self.imu_queue_size, tcp_nodelay=True)
        if self.lio_input_health_topic:
            self.health_sub = rospy.Subscriber(self.lio_input_health_topic, Bool, self.on_lio_input_health, queue_size=10, tcp_nodelay=True)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.on_odom, queue_size=self.odom_queue_size, tcp_nodelay=True)
        if self.publish_rate_hz > 0.0:
            self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
            self.publish_thread.start()
        rospy.loginfo(
            "high_rate_odom_ekf started: imu=%s odom=%s output=%s max_lio_age=%.3fs "
            "buffer=%.3fs replay=%.3fs publish_rate=%.1fHz queues=(imu=%d odom=%d pub=%d)",
            self.imu_topic,
            self.odom_topic,
            self.output_topic,
            self.max_lio_age_sec,
            self.buffer_sec,
            self.max_replay_sec,
            self.publish_rate_hz,
            self.imu_queue_size,
            self.odom_queue_size,
            self.publish_queue_size,
        )

    def snapshot(self, stamp, imu_msg):
        return {
            "stamp": stamp,
            "imu": imu_msg,
            "p": self.p.copy(),
            "v": self.v.copy(),
            "q": self.q.copy(),
            "bg": self.bg.copy(),
            "ba": self.ba.copy(),
            "P": self.P.copy(),
        }

    def restore(self, snap):
        self.p = snap["p"].copy()
        self.v = snap["v"].copy()
        self.q = snap["q"].copy()
        self.bg = snap["bg"].copy()
        self.ba = snap["ba"].copy()
        self.P = snap["P"].copy()

    def trim_buffer(self, now):
        while self.buffer and (now - self.buffer[0]["stamp"]).to_sec() > self.buffer_sec:
            self.buffer.popleft()

    def predict_with_imu(self, imu_msg, dt):
        if dt <= 0.0:
            return
        if dt > self.max_dt:
            dt = self.max_dt

        gyro = np.array(
            [imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z],
            dtype=float,
        )
        acc = np.array(
            [imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z],
            dtype=float,
        )
        self.latest_gyro = gyro - self.bg

        omega = gyro - self.bg
        acc_unbiased = acc - self.ba
        R = quat_to_rot(self.q)
        a_world = R.dot(acc_unbiased) + self.gravity

        if self.use_imu_accel_for_position:
            self.p = self.p + self.v * dt + 0.5 * a_world * dt * dt
            self.v = self.v + a_world * dt
        else:
            self.p = self.p + self.v * dt
        self.q = quat_normalize(quat_mul(self.q, delta_quat(omega * dt)))

        F = np.eye(15)
        F[0:3, 3:6] = np.eye(3) * dt
        if self.use_imu_accel_for_position:
            F[0:3, 6:9] = -0.5 * R.dot(skew(acc_unbiased)) * dt * dt
            F[0:3, 12:15] = -0.5 * R * dt * dt
            F[3:6, 6:9] = -R.dot(skew(acc_unbiased)) * dt
            F[3:6, 12:15] = -R * dt
        F[6:9, 9:12] = -np.eye(3) * dt

        Qd = np.zeros((15, 15))
        accel_var = self.process_noise[0, 0]
        gyro_var = self.process_noise[3, 3]
        gyro_bias_var = self.process_noise[6, 6]
        accel_bias_var = self.process_noise[9, 9]
        if self.use_imu_accel_for_position:
            Qd[0:3, 0:3] = np.eye(3) * (0.25 * accel_var * dt ** 4)
            Qd[3:6, 3:6] = np.eye(3) * (accel_var * dt * dt)
        else:
            Qd[0:3, 0:3] = np.eye(3) * (self.velocity_random_walk * self.velocity_random_walk * dt * dt)
            Qd[3:6, 3:6] = np.eye(3) * (self.velocity_random_walk * self.velocity_random_walk * dt)
        Qd[6:9, 6:9] = np.eye(3) * (gyro_var * dt * dt)
        Qd[9:12, 9:12] = np.eye(3) * (gyro_bias_var * dt)
        Qd[12:15, 12:15] = np.eye(3) * (accel_bias_var * dt)
        self.P = F.dot(self.P).dot(F.T) + Qd
        self.P = 0.5 * (self.P + self.P.T)

    def apply_error(self, dx):
        self.p += dx[0:3]
        self.v += dx[3:6]
        self.q = quat_normalize(quat_mul(self.q, delta_quat(dx[6:9])))
        self.bg += dx[9:12]
        self.ba += dx[12:15]

    def odom_velocity_measurement(self, msg, stamp, z_p):
        z_v = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            dtype=float,
        )
        if np.linalg.norm(z_v) > 1.0e-4:
            return z_v
        if self.last_odom_stamp is not None and self.last_odom_position is not None:
            dt = (stamp - self.last_odom_stamp).to_sec()
            if 1.0e-4 < dt < 1.0:
                derived = (z_p - self.last_odom_position) / dt
                if np.linalg.norm(derived) < 10.0:
                    return derived
        return None

    def measurement_now_ns(self, raw_ns):
        if self.timebase_offset_ns is not None:
            return raw_ns + self.timebase_offset_ns
        return rospy.Time.now().to_nsec()

    def checked_stamp(self, stamp, max_age_ns, source):
        if self.timebase_offset_ns is None and stamp.is_zero():
            stamp = rospy.Time.now()
        age_ns = self.measurement_now_ns(monotonic_raw_now_ns()) - stamp.to_nsec()
        if not stamp.is_zero() and -FUTURE_TOLERANCE_NS <= age_ns <= max_age_ns:
            return stamp
        rospy.logwarn_throttle(
            1.0, "high_rate_odom_ekf rejected %s timestamp: age %.3f s", source, age_ns / 1e9,
        )
        return None

    def mark_inputs_stale(self, source, sample_age_ns, receive_age_ns):
        self.lio_stale = True
        rospy.logwarn_throttle(
            1.0, "high_rate_odom_ekf suppressing output: stale %s (sample age %.3f s, receive age %.3f s)",
            source, sample_age_ns / 1e9, receive_age_ns / 1e9,
        )

    def publish_if_fresh(self):
        if not self.lio_frontend_ok or self.lio_stale or self.last_imu_stamp is None or self.last_odom_stamp is None:
            return
        raw_now_ns = monotonic_raw_now_ns()
        measurement_now_ns = self.measurement_now_ns(raw_now_ns)
        inputs = (
            ("LIO", self.last_odom_stamp, self.last_odom_received_ns, self.max_lio_age_ns),
            ("IMU", self.last_imu_stamp, self.last_imu_received_ns, self.max_imu_age_ns),
        )
        for source, stamp, received_ns, max_age_ns in inputs:
            sample_age_ns = measurement_now_ns - stamp.to_nsec()
            receive_age_ns = raw_now_ns - received_ns
            if not -FUTURE_TOLERANCE_NS <= sample_age_ns <= max_age_ns or receive_age_ns > max_age_ns:
                self.mark_inputs_stale(source, sample_age_ns, receive_age_ns)
                return
        self.publish(self.last_imu_stamp)

    def initialize_from_odom(self, msg, stamp, received_ns, reason):
        self.p = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )
        self.v = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            dtype=float,
        )
        self.q = msg_quat_to_array(msg.pose.pose.orientation)
        self.initialized = True
        self.lio_stale = False
        self.last_imu_stamp = None
        self.last_imu_received_ns = None
        self.last_odom_stamp = stamp
        self.last_odom_received_ns = received_ns
        self.last_odom_position = self.p.copy()
        self.buffer.clear()
        self.P = np.eye(15) * 0.01
        self.P[3:6, 3:6] *= 10.0
        rospy.loginfo("high_rate_odom_ekf %s from %s at %.3f", reason, self.odom_topic, stamp.to_sec())

    def correct_with_odom(self, msg, stamp, received_ns):
        z_p = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=float)
        z_q = msg_quat_to_array(msg.pose.pose.orientation)
        z_v = self.odom_velocity_measurement(msg, stamp, z_p)
        residual = np.zeros(6)
        residual[0:3] = z_p - self.p
        residual[3:6] = quat_log(quat_mul(quat_conj(self.q), z_q))

        pos_norm = np.linalg.norm(residual[0:3])
        if self.reject_position_threshold_m > 0.0 and pos_norm > self.reject_position_threshold_m:
            rospy.logwarn_throttle(
                1.0,
                "high_rate_odom_ekf rejected odom update: position residual %.3fm > %.3fm",
                pos_norm,
                self.reject_position_threshold_m,
            )
            return False

        H = np.zeros((6, 15))
        H[0:3, 0:3] = np.eye(3)
        H[3:6, 6:9] = np.eye(3)
        S = H.dot(self.P).dot(H.T) + self.R_meas
        K = self.P.dot(H.T).dot(np.linalg.inv(S))
        dx = K.dot(residual)
        self.apply_error(dx)
        I = np.eye(15)
        IKH = I - K.dot(H)
        self.P = IKH.dot(self.P).dot(IKH.T) + K.dot(self.R_meas).dot(K.T)
        self.P = 0.5 * (self.P + self.P.T)
        if z_v is not None:
            gain = min(max(self.odom_velocity_gain, 0.0), 1.0)
            self.v = (1.0 - gain) * self.v + gain * z_v
        self.last_odom_stamp = stamp
        self.last_odom_received_ns = received_ns
        self.last_odom_position = z_p
        return True

    def on_lio_input_health(self, msg):
        with self.lock:
            self.lio_frontend_ok = msg.data
            if not msg.data and self.initialized:
                self.lio_stale = True

    def on_odom(self, msg):
        received_ns = monotonic_raw_now_ns()
        with self.lock:
            if not self.lio_frontend_ok:
                return
            stamp = self.checked_stamp(msg.header.stamp, self.max_lio_age_ns, "LIO")
            if stamp is None or (self.last_odom_stamp is not None and stamp <= self.last_odom_stamp):
                return

            self.frame_id = self.output_frame_id or msg.header.frame_id or self.frame_id
            self.child_frame_id = self.output_child_frame_id or msg.child_frame_id or self.child_frame_id
            if not self.initialized or self.lio_stale:
                reason = "reinitialized after stale input" if self.initialized else "initialized"
                self.initialize_from_odom(msg, stamp, received_ns, reason)
                return

            if self.buffer:
                idx = None
                for i in range(len(self.buffer) - 1, -1, -1):
                    if self.buffer[i]["stamp"] <= stamp:
                        idx = i
                        break
                if idx is None:
                    rospy.logwarn_throttle(1.0, "high_rate_odom_ekf ignored odom older than replay buffer")
                    return
                replay_lag = (self.buffer[-1]["stamp"] - stamp).to_sec()
                if replay_lag <= self.max_replay_sec:
                    replay = list(self.buffer)[idx + 1 :]
                    self.restore(self.buffer[idx])
                    self.correct_with_odom(msg, stamp, received_ns)
                    previous_stamp = self.buffer[idx]["stamp"]
                    new_tail = []
                    for snap in replay:
                        dt = (snap["stamp"] - previous_stamp).to_sec()
                        self.predict_with_imu(snap["imu"], dt)
                        new_tail.append(self.snapshot(snap["stamp"], snap["imu"]))
                        previous_stamp = snap["stamp"]
                    keep = list(self.buffer)[: idx + 1] + new_tail
                    self.buffer = deque(keep)
                    return
                rospy.logwarn_throttle(
                    1.0,
                    "high_rate_odom_ekf ignored delayed odom outside replay window: lag %.3fs > %.3fs",
                    replay_lag,
                    self.max_replay_sec,
                )
                return

            self.correct_with_odom(msg, stamp, received_ns)

    def on_imu(self, msg):
        received_ns = monotonic_raw_now_ns()
        with self.lock:
            if not self.lio_frontend_ok or not self.initialized or self.lio_stale:
                return
            stamp = self.checked_stamp(msg.header.stamp, self.max_imu_age_ns, "IMU")
            if stamp is None:
                return
            if self.last_imu_stamp is not None:
                step_ns = (stamp - self.last_imu_stamp).to_nsec()
                if step_ns <= 0:
                    return
                receive_gap_ns = received_ns - self.last_imu_received_ns
                if max(step_ns, receive_gap_ns) > self.max_imu_age_ns:
                    self.mark_inputs_stale("IMU gap", step_ns, receive_gap_ns)
                    return
                self.predict_with_imu(msg, step_ns / 1e9)
            self.last_imu_stamp = stamp
            self.last_imu_received_ns = received_ns
            self.buffer.append(self.snapshot(stamp, msg))
            self.trim_buffer(stamp)
            if self.publish_thread is None:
                self.publish_if_fresh()

    def publish_loop(self):
        period = 1.0 / self.publish_rate_hz
        next_time = time.monotonic() + period
        while not rospy.is_shutdown():
            sleep_time = next_time - time.monotonic()
            if sleep_time > 0.0:
                time.sleep(sleep_time)
            else:
                next_time = time.monotonic()
            next_time += period
            with self.lock:
                self.publish_if_fresh()

    def publish(self, stamp):
        if self.last_published_stamp is not None and stamp <= self.last_published_stamp:
            return
        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id
        msg.pose.pose.position.x = self.p[0]
        msg.pose.pose.position.y = self.p[1]
        msg.pose.pose.position.z = self.p[2]
        msg.pose.pose.orientation.w = self.q[0]
        msg.pose.pose.orientation.x = self.q[1]
        msg.pose.pose.orientation.y = self.q[2]
        msg.pose.pose.orientation.z = self.q[3]
        msg.twist.twist.linear.x = self.v[0]
        msg.twist.twist.linear.y = self.v[1]
        msg.twist.twist.linear.z = self.v[2]
        msg.twist.twist.angular.x = self.latest_gyro[0]
        msg.twist.twist.angular.y = self.latest_gyro[1]
        msg.twist.twist.angular.z = self.latest_gyro[2]

        pose_cov = np.zeros((6, 6))
        pose_cov[0:3, 0:3] = self.P[0:3, 0:3]
        pose_cov[3:6, 3:6] = self.P[6:9, 6:9]
        msg.pose.covariance = tuple(pose_cov.reshape(-1).tolist())
        twist_cov = np.zeros((6, 6))
        twist_cov[0:3, 0:3] = self.P[3:6, 3:6]
        twist_cov[3:6, 3:6] = self.P[9:12, 9:12]
        msg.twist.covariance = tuple(twist_cov.reshape(-1).tolist())

        self.pub.publish(msg)
        self.last_published_stamp = stamp
        if self.tf_pub is not None:
            tf_msg = TransformStamped()
            tf_msg.header = msg.header
            tf_msg.child_frame_id = msg.child_frame_id
            tf_msg.transform.translation.x = msg.pose.pose.position.x
            tf_msg.transform.translation.y = msg.pose.pose.position.y
            tf_msg.transform.translation.z = msg.pose.pose.position.z
            tf_msg.transform.rotation = msg.pose.pose.orientation
            self.tf_pub.sendTransform(tf_msg)


def main():
    rospy.init_node("high_rate_odom_ekf")
    HighRateOdomEkf()
    rospy.spin()


if __name__ == "__main__":
    main()
