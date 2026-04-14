/***********************************************************
 *                                                         *
 * Copyright (c)                                           *
 *                                                         *
 * The Verifiable & Control-Theoretic Robotics (VECTR) Lab *
 * University of California, Los Angeles                   *
 *                                                         *
 * Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez   *
 * Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu         *
 *                                                         *
 ***********************************************************/

#include "dlio/dlio.h"
#include "std_srvs/SetBool.h"
// #include "mavros_msgs/State.h"
#include <mutex>
class dlio::MapNode {

public:

  MapNode(ros::NodeHandle node_handle);
  ~MapNode();

  void start();

private:

  void getParams();

  void callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe);
  void callbackDenseMap(const sensor_msgs::PointCloud2ConstPtr& keyframe);
  // void callbackState(const mavros_msgs::State::ConstPtr &msg);

  bool savePcd(dlio::save_pcd::Request& req,
               dlio::save_pcd::Response& res);

  bool saveLas(std_srvs::SetBool::Request& req,
               std_srvs::SetBool::Response& res);
  ros::NodeHandle nh;

  ros::Subscriber keyframe_sub;
  ros::Subscriber dense_sub;
  ros::Subscriber state_sub;
  ros::Publisher map_pub;
  // ros::Publisher dense_map_pub;

  ros::ServiceServer save_pcd_srv;
  ros::ServiceServer save_las_srv;

  pcl::PointCloud<PointType>::Ptr dlio_map;

  pcl::PointCloud<PointType>::Ptr dense_map;
  pcl::VoxelGrid<PointType> voxelgrid;

  std::string odom_frame;

  bool rc_ctrl_;
  bool last_rc_ctrl_;
  double last_timestamp_rc;

  std::mutex pcd_mtx;

  double leaf_size_;

};
