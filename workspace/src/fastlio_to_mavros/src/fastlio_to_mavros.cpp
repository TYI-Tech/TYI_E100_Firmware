#include <cmath>
#include <queue>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace {

Eigen::Vector3d p_lidar_body(0, 0, 0);
Eigen::Vector3d p_enu(0, 0, 0);
Eigen::Quaterniond q_lidar = Eigen::Quaterniond::Identity();
Eigen::Quaterniond q_px4_odom = Eigen::Quaterniond::Identity();
Eigen::Quaterniond q_init_ENU_from_LIO = Eigen::Quaterniond::Identity();
Eigen::Quaterniond q_base_from_lidar = Eigen::Quaterniond::Identity();
Eigen::Vector3d t_base_from_lidar(0, 0, 0);

ros::Time latest_lio_stamp;
ros::Time last_published_input_stamp;

bool init_flag = false;
bool has_lio_odom = false;
bool has_px4_odom = false;
bool use_lidar_extrinsic = false;

int window_size = 8;
std::string fastlio_odom_topic = "/Odometry";
std::string px4_odom_topic = "/mavros/local_position/odom";
std::string vision_pose_topic = "/mavros/vision_pose/pose";
std::string vision_frame_id = "map";
std::string stamp_source = "input";

double normalizeAngle(double angle) {
  while (angle <= -M_PI) {
    angle += 2.0 * M_PI;
  }
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  return angle;
}

double quatToYaw(const Eigen::Quaterniond& q) {
  return normalizeAngle(std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                                   1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z())));
}

class AngleUnwrapper {
 public:
  double unwrap(double angle) {
    if (!has_last_) {
      last_unwrapped_ = angle;
      has_last_ = true;
      return last_unwrapped_;
    }

    double delta = normalizeAngle(angle - normalizeAngle(last_unwrapped_));
    last_unwrapped_ += delta;
    return last_unwrapped_;
  }

 private:
  bool has_last_ = false;
  double last_unwrapped_ = 0.0;
};

class SlidingWindowAverage {
 public:
  explicit SlidingWindowAverage(int size) : size_(size) {}

  double add(double value) {
    data_.push(value);
    sum_ += value;
    if (static_cast<int>(data_.size()) > size_) {
      sum_ -= data_.front();
      data_.pop();
    }
    return sum_ / static_cast<double>(data_.size());
  }

  int size() const {
    return static_cast<int>(data_.size());
  }

  int capacity() const {
    return size_;
  }

  double avg() const {
    return data_.empty() ? 0.0 : sum_ / static_cast<double>(data_.size());
  }

 private:
  int size_;
  std::queue<double> data_;
  double sum_ = 0.0;
};

AngleUnwrapper yaw_unwrapper;
SlidingWindowAverage sliding_yaw_avg(8);

void fastlioCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  p_lidar_body = Eigen::Vector3d(msg->pose.pose.position.x,
                                 msg->pose.pose.position.y,
                                 msg->pose.pose.position.z);
  q_lidar = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z);
  latest_lio_stamp = msg->header.stamp;
  has_lio_odom = true;
}

void px4OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  q_px4_odom = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                  msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y,
                                  msg->pose.pose.orientation.z);
  sliding_yaw_avg.add(yaw_unwrapper.unwrap(quatToYaw(q_px4_odom)));
  has_px4_odom = true;
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "fastlio_to_mavros");
  ros::NodeHandle nh("~");

  nh.param("window_size", window_size, 8);
  nh.param("fastlio_topic", fastlio_odom_topic, std::string("/Odometry"));
  nh.param("px4_odom_topic", px4_odom_topic, std::string("/mavros/local_position/odom"));
  nh.param("vision_topic", vision_pose_topic, std::string("/mavros/vision_pose/pose"));
  nh.param("vision_frame_id", vision_frame_id, std::string("map"));
  nh.param("stamp_source", stamp_source, std::string("input"));
  nh.param("use_lidar_extrinsic", use_lidar_extrinsic, false);

  std::vector<double> q_ext(4, 0.0);
  std::vector<double> t_ext(3, 0.0);
  if (nh.getParam("q_base_from_lidar", q_ext) && q_ext.size() == 4) {
    q_base_from_lidar =
        Eigen::Quaterniond(q_ext[0], q_ext[1], q_ext[2], q_ext[3]).normalized();
  }
  if (nh.getParam("t_base_from_lidar", t_ext) && t_ext.size() == 3) {
    t_base_from_lidar = Eigen::Vector3d(t_ext[0], t_ext[1], t_ext[2]);
  }

  sliding_yaw_avg = SlidingWindowAverage(window_size);

  ros::Subscriber sub_lio =
      nh.subscribe<nav_msgs::Odometry>(fastlio_odom_topic, 100, fastlioCallback);
  ros::Subscriber sub_px4 =
      nh.subscribe<nav_msgs::Odometry>(px4_odom_topic, 5, px4OdomCallback);
  ros::Publisher pub_pose =
      nh.advertise<geometry_msgs::PoseStamped>(vision_pose_topic, 10);

  ros::Rate rate(30.0);
  ros::Time start_time = ros::Time::now();

  while (ros::ok()) {
    ros::spinOnce();

    if (!init_flag && has_px4_odom &&
        sliding_yaw_avg.size() >= sliding_yaw_avg.capacity() &&
        (ros::Time::now() - start_time).toSec() > 1.0) {
      double yaw_avg = normalizeAngle(sliding_yaw_avg.avg());
      q_init_ENU_from_LIO =
          Eigen::AngleAxisd(yaw_avg, Eigen::Vector3d::UnitZ());
      init_flag = true;
      ROS_INFO("[fastlio_to_mavros] Init done. yaw_avg(deg)=%.2f",
               yaw_avg * 180.0 / M_PI);
    }

    if (init_flag && has_lio_odom) {
      Eigen::Vector3d p_in_use = p_lidar_body;
      Eigen::Quaterniond q_in_use = q_lidar;
      if (use_lidar_extrinsic) {
        p_in_use = q_base_from_lidar * p_lidar_body + t_base_from_lidar;
        q_in_use = (q_base_from_lidar * q_lidar).normalized();
      }

      if (stamp_source == "input" && !latest_lio_stamp.isZero() &&
          latest_lio_stamp == last_published_input_stamp) {
        rate.sleep();
        continue;
      }

      p_enu = q_init_ENU_from_LIO * p_in_use;
      Eigen::Quaterniond q_enu =
          (q_init_ENU_from_LIO * q_in_use).normalized();

      geometry_msgs::PoseStamped vision;
      if (stamp_source == "input" && !latest_lio_stamp.isZero()) {
        vision.header.stamp = latest_lio_stamp;
        last_published_input_stamp = latest_lio_stamp;
      } else {
        vision.header.stamp = ros::Time::now();
      }
      vision.header.frame_id = vision_frame_id;
      vision.pose.position.x = p_enu.x();
      vision.pose.position.y = p_enu.y();
      vision.pose.position.z = p_enu.z();
      vision.pose.orientation.x = q_enu.x();
      vision.pose.orientation.y = q_enu.y();
      vision.pose.orientation.z = q_enu.z();
      vision.pose.orientation.w = q_enu.w();

      pub_pose.publish(vision);
      ROS_INFO_THROTTLE(1.0, "[fastlio_to_mavros] ENU pos: (%.3f %.3f %.3f)",
                        p_enu.x(), p_enu.y(), p_enu.z());
    }

    rate.sleep();
  }

  return 0;
}
