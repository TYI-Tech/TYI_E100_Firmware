/**
 * @brief VisionPoseEstimate plugin
 * @file vision_pose_estimate.cpp
 * @author M.H.Kabir <mhkabir98@gmail.com>
 * @author Vladimir Ermakov <vooon341@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2014 M.H.Kabir.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros/setpoint_mixin.h>
#include <eigen_conversions/eigen_msg.h>

#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace mavros {
namespace extra_plugins {
/**
 * @brief Vision pose estimate plugin
 *
 * Send pose estimation from various vision estimators
 * to FCU position and attitude estimators.
 *
 */
class VisionPoseEstimatePlugin : public plugin::PluginBase,
	private plugin::TF2ListenerMixin<VisionPoseEstimatePlugin> {
public:
	VisionPoseEstimatePlugin() : PluginBase(),
		sp_nh("~vision_pose"),
		tf_rate(10.0)
	{ }

	void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

		bool tf_listen;

		// tf params
		sp_nh.param("tf/listen", tf_listen, false);
		sp_nh.param<std::string>("tf/frame_id", tf_frame_id, "map");
		sp_nh.param<std::string>("tf/child_frame_id", tf_child_frame_id, "vision_estimate");
		sp_nh.param("tf/rate_limit", tf_rate, 10.0);

		if (tf_listen) {
			ROS_INFO_STREAM_NAMED("vision_pose", "Listen to vision transform " << tf_frame_id
											   << " -> " << tf_child_frame_id);
			tf2_start("VisionPoseTF", &VisionPoseEstimatePlugin::transform_cb);
		}
		else {
			std::string velocity_topic;
			if (sp_nh.getParam("world_velocity_topic", velocity_topic) && !velocity_topic.empty()) {
				sp_nh.param<std::string>("velocity_parent_frame", velocity_parent_frame, "robot/odom");
				sp_nh.param<std::string>("velocity_child_frame", velocity_child_frame, "base_link");
				velocity_sub = sp_nh.subscribe(velocity_topic, 100, &VisionPoseEstimatePlugin::velocity_cb, this,
				                               ros::TransportHints().tcpNoDelay());
				pair_timer = sp_nh.createWallTimer(ros::WallDuration(0.005), &VisionPoseEstimatePlugin::on_pair_timer, this);
			}
			vision_sub = sp_nh.subscribe("pose", 10, &VisionPoseEstimatePlugin::vision_cb, this);
			vision_cov_sub = sp_nh.subscribe("pose_cov", 10, &VisionPoseEstimatePlugin::vision_cov_cb, this);
		}
	}

	Subscriptions get_subscriptions() override
	{
		return { /* Rx disabled */ };
	}

private:
	friend class TF2ListenerMixin;
	ros::NodeHandle sp_nh;

	ros::Subscriber vision_sub;
	ros::Subscriber vision_cov_sub;
	ros::Subscriber velocity_sub;
	std::string velocity_parent_frame, velocity_child_frame;
	std::map<ros::Time, nav_msgs::Odometry::ConstPtr> velocity_samples;
	std::mutex velocity_mutex;
	std::mutex send_mutex;
	using Clock = std::chrono::steady_clock;
	struct PendingPose {
		geometry_msgs::PoseStamped::ConstPtr pose;
		Clock::time_point received;
	};
	std::deque<PendingPose> pending;
	Clock::time_point last_combined{};
	ros::WallTimer pair_timer;

	void on_pair_timer(const ros::WallTimerEvent &)
	{
		std::lock_guard<std::mutex> lock(velocity_mutex);
		flush_pending();
	}

	// Called under velocity_mutex. Preserve ordering when the two ROS callbacks race.
	void flush_pending()
	{
		while (!pending.empty()) {
			const auto pose = pending.front().pose;
			const auto now = Clock::now();
			const bool matched = velocity_samples.count(pose->header.stamp) != 0;
			if (!matched && now - pending.front().received < std::chrono::milliseconds(25) && pending.size() < 8) break;
			pending.pop_front();
			// A lone missing velocity must not toggle EKF aiding off for one frame.
			// If the source goes away, resume pose-only output within 60 ms plus one pose interval.
			if (!matched && now - last_combined < std::chrono::milliseconds(60)) continue;
			Eigen::Affine3d tr;
			tf::poseMsgToEigen(pose->pose, tr);
			ftf::Covariance6d cov{};
			send_vision_estimate(pose->header.stamp, tr, cov, matched);
		}
	}

	void velocity_cb(const nav_msgs::Odometry::ConstPtr &msg)
	{
		if (msg->header.frame_id != velocity_parent_frame || msg->child_frame_id != velocity_child_frame ||
		    msg->header.stamp.isZero()) return;
		std::lock_guard<std::mutex> lock(velocity_mutex);
		velocity_samples[msg->header.stamp] = msg;
		while (velocity_samples.size() > 100) velocity_samples.erase(velocity_samples.begin());
		if (!pending.empty()) flush_pending();
	}

	// Only the configured E100 source publishes linear twist in its parent (world) axes.
	// Pair by the original EKF sample stamp; never attach a previous speed to a new pose.
	bool send_velocity_pose(const ros::Time &stamp, const Eigen::Affine3d &tr,
	                        const Eigen::Vector3d &position, const Eigen::Quaterniond &attitude,
	                        const ftf::Covariance6d &pose_cov)
	{
		const auto it = velocity_samples.find(stamp);
		if (it == velocity_samples.end()) return false;
		const auto &source = it->second;
		const auto &q = source->pose.pose.orientation;
		const Eigen::Quaterniond body_to_world(q.w, q.x, q.y, q.z);
		const auto &v = source->twist.twist.linear;
		const Eigen::Vector3d velocity(v.x, v.y, v.z);
		const ftf::EigenMapConstCovariance6d covariance(source->twist.covariance.data());
		const Eigen::Matrix3d linear_cov = covariance.topLeftCorner<3,3>();
		if (!velocity.allFinite() || !body_to_world.coeffs().allFinite() ||
		    std::abs(body_to_world.norm() - 1.0) > 1e-3 || !linear_cov.allFinite() ||
		    !linear_cov.isApprox(linear_cov.transpose()) || linear_cov.llt().info() != Eigen::Success) return false;

		// The configured pose bridge applies only a fixed world rotation about the same body origin.
		const Eigen::Matrix3d world_to_enu = tr.linear() * body_to_world.normalized().toRotationMatrix().transpose();
		Eigen::Matrix3d world_to_ned;
		world_to_ned.row(0) = world_to_enu.row(1);
		world_to_ned.row(1) = world_to_enu.row(0);
		world_to_ned.row(2) = -world_to_enu.row(2);
		const Eigen::Vector3d ned = world_to_ned * velocity;
		const Eigen::Matrix3d ned_cov = world_to_ned * linear_cov * world_to_ned.transpose();

		mavlink::common::msg::ODOMETRY msg{};
		msg.time_usec = stamp.toNSec() / 1000;
		msg.frame_id = msg.child_frame_id = static_cast<uint8_t>(mavlink::common::MAV_FRAME::LOCAL_NED);
		msg.estimator_type = static_cast<uint8_t>(mavlink::common::MAV_ESTIMATOR_TYPE::VISION);
		msg.x = position.x(); msg.y = position.y(); msg.z = position.z();
		ftf::quaternion_to_mavlink(attitude, msg.q);
		msg.vx = ned.x(); msg.vy = ned.y(); msg.vz = ned.z();
		msg.rollspeed = msg.pitchspeed = msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
		ftf::EigenMapConstCovariance6d pose_cov_map(pose_cov.data());
		ftf::covariance_urt_to_mavlink(pose_cov_map, msg.pose_covariance);
		msg.velocity_covariance.fill(std::numeric_limits<float>::quiet_NaN());
		for (int i = 0; i < 3; ++i) {
			for (int k = i; k < 3; ++k) msg.velocity_covariance[i * (13 - i) / 2 + k - i] = ned_cov(i, k);
		}
		UAS_FCU(m_uas)->send_message_ignore_drop(msg);
		last_combined = Clock::now();
		return true;
	}

	std::string tf_frame_id;
	std::string tf_child_frame_id;
	double tf_rate;
	ros::Time last_transform_stamp;

	/* -*- low-level send -*- */
	/**
	 * @brief Send vision estimate transform to FCU position controller
	 */
	void send_vision_estimate(const ros::Time &stamp, const Eigen::Affine3d &tr, const geometry_msgs::PoseWithCovariance::_covariance_type &cov, bool pair_speed = false)
	{
		/**
		 * @warning Issue #60.
		 * This now affects pose callbacks too.
		 */
		std::lock_guard<std::mutex> lock(send_mutex);
		if (last_transform_stamp == stamp) {
			ROS_DEBUG_THROTTLE_NAMED(10, "vision_pose", "Vision: Same transform as last one, dropped.");
			return;
		}
		last_transform_stamp = stamp;

		auto position = ftf::transform_frame_enu_ned(Eigen::Vector3d(tr.translation()));
		auto attitude = ftf::transform_orientation_enu_ned(
			ftf::transform_orientation_baselink_aircraft(Eigen::Quaterniond(tr.rotation())));
		auto rpy = ftf::quaternion_to_rpy(attitude);

		auto cov_ned = ftf::transform_frame_enu_ned(cov);
		if (pair_speed && send_velocity_pose(stamp, tr, position, attitude, cov_ned)) return;
		ftf::EigenMapConstCovariance6d cov_map(cov_ned.data());

		auto urt_view = Eigen::Matrix<double, 6, 6>(cov_map.triangularView<Eigen::Upper>());
		ROS_DEBUG_STREAM_NAMED("vision_pose", "Vision: Covariance URT: " << std::endl << urt_view);

		mavlink::common::msg::VISION_POSITION_ESTIMATE vp{};

		vp.usec = stamp.toNSec() / 1000;
		// [[[cog:
		// for f in "xyz":
		//     cog.outl("vp.%s = position.%s();" % (f, f))
		// for a, b in zip("xyz", ('roll', 'pitch', 'yaw')):
		//     cog.outl("vp.%s = rpy.%s();" % (b, a))
		// ]]]
		vp.x = position.x();
		vp.y = position.y();
		vp.z = position.z();
		vp.roll = rpy.x();
		vp.pitch = rpy.y();
		vp.yaw = rpy.z();
		// [[[end]]] (sum: IEja9BF4CE)

		// just the URT of the 6x6 Pose Covariance Matrix, given
		// that the matrix is symmetric
		ftf::covariance_urt_to_mavlink(cov_map, vp.covariance);

		UAS_FCU(m_uas)->send_message_ignore_drop(vp);
	}

	/* -*- callbacks -*- */

	/* common TF listener moved to mixin */

	void transform_cb(const geometry_msgs::TransformStamped &transform)
	{
		Eigen::Affine3d tr;
		tf::transformMsgToEigen(transform.transform, tr);
		ftf::Covariance6d cov {};	// zero initialized

		send_vision_estimate(transform.header.stamp, tr, cov);
	}

	void vision_cb(const geometry_msgs::PoseStamped::ConstPtr &req)
	{
		if (velocity_sub && req->header.frame_id == "map") {
			std::lock_guard<std::mutex> lock(velocity_mutex);
			pending.push_back({req, Clock::now()});
			flush_pending();
			return;
		}
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose, tr);
		ftf::Covariance6d cov {};	// zero initialized

		send_vision_estimate(req->header.stamp, tr, cov);
	}

	void vision_cov_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &req)
	{
		Eigen::Affine3d tr;
		tf::poseMsgToEigen(req->pose.pose, tr);

		send_vision_estimate(req->header.stamp, tr, req->pose.covariance);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::VisionPoseEstimatePlugin, mavros::plugin::PluginBase)
