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

#include "dlio/map.h"


dlio::MapNode::MapNode(ros::NodeHandle node_handle) : nh(node_handle) {

  this->getParams();

  this->keyframe_sub = this->nh.subscribe("keyframes", 10,
      &dlio::MapNode::callbackKeyframe, this, ros::TransportHints().tcpNoDelay());
  this->dense_sub = this->nh.subscribe("/robot/dlio_odom/deskewed", 10,
      &dlio::MapNode::callbackDenseMap, this, ros::TransportHints().tcpNoDelay());

  // this->state_sub = this->nh.subscribe("/mavros/state", 10,
  //     &dlio::MapNode::callbackState, this, ros::TransportHints().tcpNoDelay());
  this->map_pub = this->nh.advertise<sensor_msgs::PointCloud2>("map", 100);



  this->save_pcd_srv = this->nh.advertiseService("save_pcd", &dlio::MapNode::savePcd, this);
  this->save_las_srv = this->nh.advertiseService("save_las", &dlio::MapNode::saveLas, this);

  this->dlio_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  this->dense_map = pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());


  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

}

dlio::MapNode::~MapNode() {}

void dlio::MapNode::getParams() {

  ros::param::param<std::string>("~dlio/odom/odom_frame", this->odom_frame, "odom");
  ros::param::param<double>("~dlio/map/sparse/leafSize", this->leaf_size_, 0.5);

  // Get Node NS and Remove Leading Character
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,1);

  // Concatenate Frame Name Strings
  this->odom_frame = ns + "/" + this->odom_frame;

}

void dlio::MapNode::start() {
}

void dlio::MapNode::callbackKeyframe(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  // voxel filter
  this->voxelgrid.setLeafSize(this->leaf_size_, this->leaf_size_, this->leaf_size_);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  *this->dlio_map += *keyframe_pcl;

  // publish full map
  if (this->dlio_map->points.size() == this->dlio_map->width * this->dlio_map->height) {
    sensor_msgs::PointCloud2 map_ros;
    pcl::toROSMsg(*this->dlio_map, map_ros);
    map_ros.header.stamp = ros::Time::now();
    map_ros.header.frame_id = this->odom_frame;
    this->map_pub.publish(map_ros);
  }

}


void dlio::MapNode::callbackDenseMap(const sensor_msgs::PointCloud2ConstPtr& keyframe) {

  // convert scan to pcl format
  pcl::PointCloud<PointType>::Ptr keyframe_pcl =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>());
  pcl::fromROSMsg(*keyframe, *keyframe_pcl);

  this->voxelgrid.setLeafSize(0.05, 0.05, 0.05);
  this->voxelgrid.setInputCloud(keyframe_pcl);
  this->voxelgrid.filter(*keyframe_pcl);

  // save filtered keyframe to map for rviz
  pcd_mtx.lock();
  *this->dense_map += *keyframe_pcl;
  pcd_mtx.unlock();

  // publish full map
  // if (this->dense_map->points.size() == this->dense_map->width * this->dense_map->height) {
  //   sensor_msgs::PointCloud2 map_ros;
  //   pcl::toROSMsg(*this->dense_map, map_ros);
  //   map_ros.header.stamp = ros::Time::now();
  //   map_ros.header.frame_id = this->odom_frame;
  //   this->map_pub.publish(map_ros);
  // }

}



bool dlio::MapNode::savePcd(dlio::save_pcd::Request& req,
                            dlio::save_pcd::Response& res) {

  pcl::PointCloud<PointType>::Ptr m =
    pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dlio_map));

  float leaf_size = req.leaf_size;
  std::string p = req.save_path;

  std::cout << std::setprecision(2) << "Saving map to " << p + "/dlio_map.pcd"
    << " with leaf size " << to_string_with_precision(leaf_size, 2) << "... "; std::cout.flush();

  // voxelize map
  // pcl::VoxelGrid<PointType> vg;
  // vg.setLeafSize(leaf_size, leaf_size, leaf_size);
  // vg.setInputCloud(m);
  // vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary(p + "/dlio_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }

  return res.success;

}

bool dlio::MapNode::saveLas(std_srvs::SetBool::Request& req,
               std_srvs::SetBool::Response& res) {
  pcd_mtx.lock();
  pcl::PointCloud<PointType>::Ptr m =
  pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dense_map));
  pcd_mtx.unlock();

  // float leaf_size = req.leaf_size;
  // std::string p = req.save_path;
  if(req.data == true)
  {

  // voxelize map
  // pcl::VoxelGrid<PointType> vg;
  // vg.setLeafSize(0.1, 0.1, 0.1);
  // vg.setInputCloud(m);
  // vg.filter(*m);

  // save map
  int ret = pcl::io::savePCDFileBinary("/workspace/catkin_ws/src/dlio/maps/dlio_map.pcd", *m);
  res.success = ret == 0;

  if (res.success) {
    std::cout << "done" << std::endl;
  } else {
    std::cout << "failed" << std::endl;
  }
  
 }
 else
  res.success = false;
return res.success;
 

  }

// void dlio::MapNode::callbackState(const mavros_msgs::State::ConstPtr &msg)
// {
// //     mtx_buffer.lock();
//     if(msg->header.stamp.toSec() > last_timestamp_rc) rc_ctrl_ = msg->armed;
//     else ROS_ERROR("RC time errors");
// //    cout<<"rc_ctrl:  "<<rc_ctrl_<<endl;
//     if(rc_ctrl_ == last_rc_ctrl_ && last_rc_ctrl_ == false)
//     {
//         // pcl_wait_save->clear();
//     }
//     // else pcd_save_en=true;
//     if(rc_ctrl_ == false && last_rc_ctrl_ == true)
//     {

//                 std::string all_points_dir("/media/rpdzkj/F400Lidar/test.pcd");
// //                std::cout<<"Pcd_index:"<<pcd_index<<endl;
//                 pcd_mtx.lock();
//                 pcl::PointCloud<PointType>::Ptr m =
//                 pcl::PointCloud<PointType>::Ptr (boost::make_shared<pcl::PointCloud<PointType>>(*this->dense_map));
//                 pcd_mtx.unlock();
//                 // pcl::PCDWriter pcd_writer;
//                 // pcd_writer.writeBinary(all_points_dir,*pcl_wait_save);
//                 pcl::io::savePCDFileBinary(all_points_dir, *m);
        
//         // pcd_save_en = false;
//         // pcl_wait_save->clear();
//     }
//     last_rc_ctrl_ = rc_ctrl_;
//     last_timestamp_rc = msg->header.stamp.toSec();
// }
