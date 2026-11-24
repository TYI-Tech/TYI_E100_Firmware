// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <algorithm>
#include <chrono>
#include <boost/make_shared.hpp>
#include <memory>
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "incremental_voxel_map.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;
string odom_topic = "/Odometry";
string path_topic = "/path";
string cloud_topic = "/cloud_registered";
string body_cloud_topic = "/cloud_registered_body";
string effect_cloud_topic = "/cloud_effected";
string map_cloud_topic = "/Laser_map";
string odom_frame_id = "camera_init";
string body_frame_id = "body";
int publish_queue_size = 10;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
int lidar_type;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
using InputClock = std::chrono::steady_clock;
struct TimedLidarScan
{
    double begin, end;
    InputClock::time_point received;
    PointCloudXYZI::Ptr cloud;
};

constexpr std::size_t kMaxPendingScans = 4;
constexpr std::size_t kMaxImuSamples = 1000;
constexpr double kImuHistorySeconds = 2.0;
constexpr int kRecoveryFrames = 2;
std::deque<TimedLidarScan> lidar_buffer;
std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
InputClock::time_point last_lidar_receive, last_imu_receive;
double input_imu_timeout = 0.1, input_lidar_timeout = 0.2, max_imu_gap = 0.02;
double last_filter_time = -1.0;
bool input_healthy = false;
int consecutive_good_frames = 0;
ros::Publisher input_health_pub;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;

pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

KD_TREE<PointType> ikdtree;
using VoxelMap = tyi_lio::IncrementalVoxelMap<PointType, NUM_MATCH_POINTS>;
std::unique_ptr<VoxelMap> ivox_map;
std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> fitted_planes;
int match_threads = MP_PROC_NUM;
bool reuse_fitted_planes = false;
bool profile_frames = false;

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    if (ivox_map) return;
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void SetInputHealthy(bool healthy)
{
    if (input_healthy == healthy) return;
    input_healthy = healthy;
    std_msgs::Bool msg;
    msg.data = healthy;
    input_health_pub.publish(msg);
    if (healthy) ROS_INFO("FastLIO input guard: valid odometry resumed");
}

void InputFault(const char* reason)
{
    consecutive_good_frames = 0;
    if (input_healthy)
    {
        ROS_WARN("FastLIO input guard: %s", reason);
        SetInputHealthy(false);
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "FastLIO input guard: %s", reason);
    }
}

void QueueLidarScan(double begin, double span, PointCloudXYZI::Ptr cloud, InputClock::time_point received)
{
    if (cloud->empty() || !std::isfinite(span) || span < 0 || span > input_lidar_timeout)
    {
        InputFault("invalid LiDAR frame");
        return;
    }

    if (lidar_buffer.size() >= kMaxPendingScans)
    {
        lidar_buffer.pop_front();
        InputFault("LiDAR queue overflow; discarded oldest scan");
    }
    last_lidar_receive = received;
    last_timestamp_lidar = begin;
    lidar_buffer.push_back({begin, begin + span, received, std::move(cloud)});
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    const auto received = InputClock::now();
    const double begin = msg->header.stamp.toSec();
    if (begin <= 0 || begin <= last_timestamp_lidar)
    {
        InputFault("non-increasing LiDAR stamp");
        return;
    }
    std::lock_guard<std::mutex> lock(mtx_buffer);
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre->process(msg, cloud);
    double span = 0.0;
    if (lidar_type != MARSIM)
    {
        for (const auto& point : cloud->points)
        {
            if (!std::isfinite(point.curvature) || point.curvature < 0)
            {
                InputFault("invalid LiDAR point offset");
                return;
            }
            span = std::max(span, point.curvature * 1e-3);
        }
    }
    QueueLidarScan(begin, span, cloud, received);
}

double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
    const auto received = InputClock::now();
    const double begin = msg->header.stamp.toSec();
    if (begin <= 0 || begin <= last_timestamp_lidar)
    {
        InputFault("non-increasing LiDAR stamp");
        return;
    }
    if (msg->point_num != msg->points.size() || msg->points.size() < 2)
    {
        InputFault("invalid Livox point count");
        return;
    }

    std::lock_guard<std::mutex> lock(mtx_buffer);
    if (time_sync_en && !timediff_set_flg && std::abs(last_timestamp_imu - begin) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = begin + 0.1 - last_timestamp_imu;
        imu_buffer.clear();
        last_timestamp_imu = -1.0;
        InputFault("LiDAR-IMU time offset changed; discarded old IMU window");
    }

    const double preprocess_start = omp_get_wtime();
    uint32_t max_offset_ns = 0;
    for (const auto& point : msg->points)
        max_offset_ns = std::max(max_offset_ns, point.offset_time);
    if (max_offset_ns == 0 || max_offset_ns * 1e-9 > input_lidar_timeout)
    {
        InputFault("invalid Livox frame span");
        return;
    }
    PointCloudXYZI::Ptr cloud(new PointCloudXYZI());
    p_pre->process(msg, cloud);
    if (scan_count < MAXN - 1)
        s_plot11[++scan_count] = omp_get_wtime() - preprocess_start;
    QueueLidarScan(begin, max_offset_ns * 1e-9, cloud, received);
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    const auto received = InputClock::now();
    const auto& a = msg_in->linear_acceleration;
    const auto& w = msg_in->angular_velocity;
    if (msg_in->header.stamp.isZero() ||
        !(std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z) &&
          std::isfinite(w.x) && std::isfinite(w.y) && std::isfinite(w.z)))
    {
        InputFault("invalid IMU measurement");
        return;
    }

    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    const double offset = time_sync_en && timediff_set_flg
                              ? timediff_lidar_wrt_imu : -time_diff_lidar_to_imu;
    if (offset != 0.0) msg->header.stamp.fromSec(msg_in->header.stamp.toSec() + offset);
    const double stamp = msg->header.stamp.toSec();
    std::lock_guard<std::mutex> lock(mtx_buffer);

    if (stamp <= last_timestamp_imu)
    {
        InputFault("non-increasing IMU stamp");
        return;
    }
    if (last_timestamp_imu > 0 && stamp - last_timestamp_imu > max_imu_gap)
        InputFault("gap in IMU timestamps");

    last_timestamp_imu = stamp;
    last_imu_receive = received;
    imu_buffer.push_back(msg);
    while (imu_buffer.size() > 2 &&
           (imu_buffer.size() > kMaxImuSamples ||
            stamp - imu_buffer.front()->header.stamp.toSec() > kImuHistorySeconds))
        imu_buffer.pop_front();
}

using ImuIterator = std::deque<sensor_msgs::Imu::ConstPtr>::const_iterator;

bool FindImuWindow(double start, double end, ImuIterator& left, ImuIterator& right)
{
    if (imu_buffer.empty() || start > end ||
        imu_buffer.front()->header.stamp.toSec() > start ||
        imu_buffer.back()->header.stamp.toSec() < end) return false;

    left = std::prev(std::upper_bound(imu_buffer.cbegin(), imu_buffer.cend(), start,
                                    [](double t, const sensor_msgs::Imu::ConstPtr& sample) {
                                        return t < sample->header.stamp.toSec();
                                    }));
    right = std::lower_bound(left, imu_buffer.cend(), end,
                             [](const sensor_msgs::Imu::ConstPtr& sample, double t) {
                                 return sample->header.stamp.toSec() < t;
                             });
    for (auto it = left; it != right; ++it)
    {
        if ((*std::next(it))->header.stamp.toSec() - (*it)->header.stamp.toSec() > max_imu_gap)
            return false;
    }
    return true;
}

sensor_msgs::Imu::ConstPtr InterpolateImu(double stamp, const sensor_msgs::Imu::ConstPtr& earlier,
                                         const sensor_msgs::Imu::ConstPtr& later)
{
    const double start = earlier->header.stamp.toSec();
    const double end = later->header.stamp.toSec();
    if (stamp == start) return earlier;
    if (stamp == end) return later;
    const double weight = (stamp - start) / (end - start);
    const auto blend = [weight](double a, double b) { return a + weight * (b - a); };
    auto sample = boost::make_shared<sensor_msgs::Imu>(*earlier);
    sample->header.stamp.fromSec(stamp);
    sample->linear_acceleration.x = blend(earlier->linear_acceleration.x, later->linear_acceleration.x);
    sample->linear_acceleration.y = blend(earlier->linear_acceleration.y, later->linear_acceleration.y);
    sample->linear_acceleration.z = blend(earlier->linear_acceleration.z, later->linear_acceleration.z);
    sample->angular_velocity.x = blend(earlier->angular_velocity.x, later->angular_velocity.x);
    sample->angular_velocity.y = blend(earlier->angular_velocity.y, later->angular_velocity.y);
    sample->angular_velocity.z = blend(earlier->angular_velocity.z, later->angular_velocity.z);
    return sample;
}

bool sync_packages(MeasureGroup &meas, bool& reanchor)
{
    const auto now = InputClock::now();
    const auto age = [now](InputClock::time_point time) {
        return std::chrono::duration<double>(now - time).count();
    };
    if (age(last_imu_receive) > input_imu_timeout)
    {
        lidar_buffer.clear();
        InputFault("IMU receive timeout");
        return false;
    }
    if (age(last_lidar_receive) > input_lidar_timeout)
    {
        lidar_buffer.clear();
        InputFault("LiDAR receive timeout");
        return false;
    }
    if (lidar_buffer.empty() || imu_buffer.empty()) return false;

    const auto& scan = lidar_buffer.front();
    if (age(scan.received) > input_lidar_timeout ||
        scan.end <= last_filter_time ||
        (last_filter_time > scan.begin && last_filter_time - scan.begin > max_imu_gap))
    {
        lidar_buffer.pop_front();
        InputFault("expired or overlapping LiDAR frame");
        return false;
    }
    if (imu_buffer.back()->header.stamp.toSec() < scan.end) return false;

    ImuIterator left, right;
    if (!FindImuWindow(scan.begin, scan.end, left, right))
    {
        lidar_buffer.pop_front();
        InputFault("IMU does not continuously cover LiDAR frame");
        return false;
    }
    double start = last_filter_time > 0 ? last_filter_time : scan.begin;
    reanchor = last_filter_time > 0 && !FindImuWindow(start, scan.end, left, right);
    if (reanchor)
    {
        // Reacquire against the existing map. Never integrate through the gap.
        start = scan.begin;
        FindImuWindow(start, scan.end, left, right);
        InputFault("IMU propagation history interrupted; reanchoring at LiDAR frame start");
    }

    meas.imu.clear();
    if ((*left)->header.stamp.toSec() == start)
        meas.imu.push_back(*left);
    else
        meas.imu.push_back(InterpolateImu(start, *left, *std::next(left)));
    if (start < scan.end)
    {
        for (auto it = std::next(left); it != right; ++it)
            meas.imu.push_back(*it);
        if ((*right)->header.stamp.toSec() == scan.end)
            meas.imu.push_back(*right);
        else
            meas.imu.push_back(InterpolateImu(scan.end, *std::prev(right), *right));
    }
    meas.lidar = scan.cloud;
    meas.lidar_beg_time = scan.begin;
    meas.lidar_end_time = lidar_end_time = scan.end;
    lidar_buffer.pop_front();
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    if (ivox_map)
    {
        ivox_map->AddPoints(PointToAdd);
        ivox_map->AddPoints(PointNoNeedDownsample);
    }
    else
    {
        ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
    }
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = odom_frame_id;
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = body_frame_id;
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = odom_frame_id;
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = odom_frame_id;
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = odom_frame_id;
    odomAftMapped.child_frame_id = body_frame_id;
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, odom_frame_id, body_frame_id ) );
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = odom_frame_id;

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    const Eigen::Matrix3d rotation = s.rot.toRotationMatrix();
    const Eigen::Matrix3d lidar_rotation = rotation * s.offset_R_L_I.toRotationMatrix();
    const V3D lidar_translation = rotation * s.offset_T_L_I + s.pos;

    /** closest surface search and residual computation **/
    #ifdef _OPENMP
        #pragma omp parallel for schedule(static) num_threads(match_threads) if(match_threads > 1)
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        const V3D p_global = lidar_rotation * p_body + lidar_translation;
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            vector<float> point_search_sq_dis;
            if (ivox_map)
                ivox_map->Nearest(point_world, points_near, point_search_sq_dis, 5.0f);
            else
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, point_search_sq_dis);
            point_selected_surf[i] = points_near.size() >= NUM_MATCH_POINTS &&
                                    point_search_sq_dis[NUM_MATCH_POINTS - 1] <= 5.0f;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        bool plane_valid = true;
        if (reuse_fitted_planes && !ekfom_data.converge)
        {
            pabcd = fitted_planes[i];
        }
        else
        {
            plane_valid = esti_plane(pabcd, points_near, 0.1f);
            if (reuse_fitted_planes && plane_valid) fitted_planes[i] = pabcd;
        }
        point_selected_surf[i] = false;
        if (plane_valid)
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<string>("publish/odom_topic", odom_topic, string("/Odometry"));
    nh.param<string>("publish/path_topic", path_topic, string("/path"));
    nh.param<string>("publish/cloud_topic", cloud_topic, string("/cloud_registered"));
    nh.param<string>("publish/body_cloud_topic", body_cloud_topic, string("/cloud_registered_body"));
    nh.param<string>("publish/effect_cloud_topic", effect_cloud_topic, string("/cloud_effected"));
    nh.param<string>("publish/map_cloud_topic", map_cloud_topic, string("/Laser_map"));
    nh.param<string>("publish/odom_frame_id", odom_frame_id, string("camera_init"));
    nh.param<string>("publish/body_frame_id", body_frame_id, string("body"));
    nh.param<int>("publish/queue_size", publish_queue_size, 10);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<double>("mapping/fov_degree",fov_deg,180);
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<double>("frontend/imu_timeout_sec", input_imu_timeout, 0.1);
    nh.param<double>("frontend/lidar_timeout_sec", input_lidar_timeout, 0.2);
    nh.param<double>("frontend/max_imu_gap_sec", max_imu_gap, 0.02);
    if (!(max_imu_gap > 0 && max_imu_gap <= input_imu_timeout &&
          input_imu_timeout < kImuHistorySeconds &&
          input_lidar_timeout > 0 && input_lidar_timeout < kImuHistorySeconds))
    {
        ROS_FATAL("Invalid FastLIO frontend timeout configuration");
        return 1;
    }
    nh.param<int>("mapping/match_threads", match_threads, match_threads);
    match_threads = std::max(1, std::min(match_threads, omp_get_num_procs()));
    nh.param<bool>("mapping/reuse_fitted_planes", reuse_fitted_planes, false);
    nh.param<bool>("mapping/profile_frames", profile_frames, false);
    std::string map_backend;
    nh.param<std::string>("mapping/map_backend", map_backend, "ikd_tree");
    if (map_backend == "ivox")
    {
        double resolution = 0.5;
        int nearby = 18, max_voxels = 200000, max_points = 12;
        nh.param<double>("mapping/ivox_resolution", resolution, resolution);
        nh.param<int>("mapping/ivox_nearby", nearby, nearby);
        nh.param<int>("mapping/ivox_max_voxels", max_voxels, max_voxels);
        nh.param<int>("mapping/ivox_max_points_per_voxel", max_points, max_points);
        if (!std::isfinite(resolution) || resolution <= 0 ||
            (nearby != 6 && nearby != 18 && nearby != 26) ||
            max_voxels <= 0 || max_points < NUM_MATCH_POINTS)
        {
            ROS_FATAL("Invalid FastLIO iVox parameters");
            return 1;
        }
        ivox_map = std::make_unique<VoxelMap>(resolution, nearby, max_voxels, max_points);
    }
    else if (map_backend != "ikd_tree")
    {
        ROS_FATAL("Unknown FastLIO map backend: %s", map_backend.c_str());
        return 1;
    }
    ROS_INFO("FastLIO map=%s match_threads=%d reuse_fitted_planes=%s",
             map_backend.c_str(), match_threads, reuse_fitted_planes ? "true" : "false");
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    p_pre->lidar_type = lidar_type;
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id = odom_frame_id;

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = lidar_type;
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    input_health_pub = nh.advertise<std_msgs::Bool>(odom_topic + "/input_healthy", 1, true);
    input_health_pub.publish(std_msgs::Bool());
    last_lidar_receive = last_imu_receive = InputClock::now();
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ?
        nh.subscribe(lid_topic, kMaxPendingScans, livox_pcl_cbk) :
        nh.subscribe(lid_topic, kMaxPendingScans, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, kMaxImuSamples, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            (cloud_topic, publish_queue_size);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            (body_cloud_topic, publish_queue_size);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            (effect_cloud_topic, publish_queue_size);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            (map_cloud_topic, publish_queue_size);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            (odom_topic, publish_queue_size);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            (path_topic, publish_queue_size);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        bool reanchor = false;
        if (sync_packages(Measures, reanchor))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();

            if (reanchor) p_imu->Reanchor(Measures.imu.front(), kf.get_x());
            p_imu->Process(Measures, kf, feats_undistort);
            last_filter_time = Measures.lidar_end_time;
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if (ivox_map ? ivox_map->empty() : ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    if (ivox_map)
                    {
                        ivox_map->AddPoints(feats_down_world->points);
                    }
                    else
                    {
                        ikdtree.set_downsample_param(filter_size_map_min);
                        ikdtree.Build(feats_down_world->points);
                    }
                }
                continue;
            }
            kdtree_size_st = ivox_map ? ivox_map->size() : ikdtree.size();
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            if (runtime_pos_log)
            {
                const V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_pre << setw(20) << Measures.lidar_beg_time - first_lidar_time
                         << " " << euler_cur.transpose() << " " << state_point.pos.transpose()
                         << " " << ext_euler.transpose() << " " << state_point.offset_T_L_I.transpose()
                         << " " << state_point.vel.transpose() << " " << state_point.bg.transpose()
                         << " " << state_point.ba.transpose() << " " << state_point.grav << endl;
            }

            if(0) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }

            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            if (reuse_fitted_planes) fitted_planes.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];

            double t_update_end = omp_get_wtime();

            if (effct_feat_num == 0)
            {
                InputFault("no effective map correspondences");
                continue;
            }

            /******* Publish odometry *******/
            if (consecutive_good_frames < kRecoveryFrames) ++consecutive_good_frames;
            if (consecutive_good_frames >= kRecoveryFrames)
            {
                SetInputHealthy(true);
                publish_odometry(pubOdomAftMapped);
            }

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();
            t5 = omp_get_wtime();
            if (profile_frames)
            {
                ROS_INFO("TYI_LIO_PROFILE %.6f %.3f %.3f %.3f %.3f %d %d",
                         lidar_end_time, match_time * 1e3, (t_update_end - t_update_start) * 1e3,
                         (t5 - t3) * 1e3, (t5 - t0) * 1e3, effct_feat_num, kdtree_size_st);
            }
            
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;
                kdtree_size_end = ivox_map ? ivox_map->size() : ikdtree.size();
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;
                s_plot2[time_log_counter] = feats_undistort->points.size();
                s_plot3[time_log_counter] = kdtree_incremental_time;
                s_plot4[time_log_counter] = kdtree_search_time;
                s_plot5[time_log_counter] = kdtree_delete_counter;
                s_plot6[time_log_counter] = kdtree_delete_time;
                s_plot7[time_log_counter] = kdtree_size_st;
                s_plot8[time_log_counter] = kdtree_size_end;
                s_plot9[time_log_counter] = aver_time_consu;
                s_plot10[time_log_counter] = add_point_size;
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                const V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
