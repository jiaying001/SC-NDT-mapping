#define OUTPUT  // If you want to output "position_log.txt", "#define OUTPUT".

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <memory>
#include <pthread.h>
#include <chrono>
#include <mutex>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include <pcl/registration/ndt.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <time.h>
#include "lidar_localizer/save_map.h"
#include "Scancontext.h"

using namespace gtsam;

struct pose
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};

struct PointXYZIRPYT
{
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

using PointTPose = PointXYZIRPYT;
class ndt_mapping
{
public:
  ndt_mapping(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~ndt_mapping();
  void run();
  void visualThread();
  void loopClosureThread();

private:

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber points_sub_;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom;

 // struct pose{double x,y,z;double roll,pitch,yaw;};
 // struct pose current_pose_,current_pose_imu_;
 //struct pose previous_pose_;
  pose current_pose_,current_pose_imu_,previous_pose_,added_pose, pre_keypose,guess_pose,guess_pose_for_ndt;
  pose guess_pose_odom,current_pose_odom;
  double offset_odom_x, offset_odom_y, offset_odom_z, offset_odom_roll, offset_odom_pitch, offset_odom_yaw;

  pcl::PointCloud<pcl::PointXYZI> map_;
  pcl::PointCloud<pcl::PointXYZI> scan_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_keyposes_3d_;
  pcl::PointCloud<PointTPose>::Ptr cloud_keyposes_6d_;
  
  pcl::PointCloud<pcl::PointXYZI>::Ptr copy_cloudKeyPoses2D; // sc
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRaw; //  sc
  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudRawDS; //  sc
  double laserCloudRawTime;

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloud_keyframes_;

  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_poses_;

  std::vector<int> search_idx_;
  std::vector<float> search_dist_;

  pcl::VoxelGrid<pcl::PointXYZI> ds_source_;
  pcl::VoxelGrid<pcl::PointXYZI> ds_history_keyframes_;


  // Default values
  int max_iters_ ;        // Maximum iterations
  double ndt_res_ ;      // Resolution
  double step_size_ ;   // Step size
  double trans_eps_ ;  // Transformation epsilon

  double voxel_leaf_size_;// Leaf size of VoxelGrid filter.

  double scan_rate_;
  double min_scan_range_;
  double max_scan_range_;
  bool use_imu_;
  bool use_odom_;
  bool loop_closure_enabled_;
  bool savePCD_;

  std::string save_dir_;
  std::ofstream namefilepose;

  ros::Time current_scan_time;

  std::string robot_frame_;
  std::string map_frame_;

  ros::Publisher ndt_map_pub_; 
  ros::Publisher current_pose_pub_;
  ros::Publisher pub_keyposes_;
  ros::Publisher pub_recent_keyframes_;
  ros::Publisher pub_laser_cloud_surround_;
  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;

  ros::ServiceServer srv_save_map_;
  nav_msgs::Odometry current_pose_msg_;

  tf::TransformBroadcaster br_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ptr;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan_ptr;

  int initial_scan_loaded;
  double min_add_scan_shift_;
  double min_add_angle_shift_;

  double _tf_x, _tf_y, _tf_z, _tf_roll, _tf_pitch, _tf_yaw;
  Eigen::Matrix4f tf_btol_, tf_ltob_;//base_link2localizer等の略?

  bool incremental_voxel_update_;
  
  bool is_first_map_;

    // gtsam
  NonlinearFactorGraph gtSAMgraph_;
  Values initial_estimate_;
  ISAM2 *isam;
  Values isam_current_estimate_;

  noiseModel::Diagonal::shared_ptr prior_noise_;
  noiseModel::Diagonal::shared_ptr odom_noise_;
  noiseModel::Diagonal::shared_ptr constraint_noise_;
  noiseModel::Base::shared_ptr robustNoiseModel;
  // loop detector 
  SCManager scManager;

  //loop closure
  int surround_search_num_; 
  double history_search_radius_; 
  int history_search_num_;
  double history_fitness_score_;
  double ds_history_size_;
  bool loop_closed_;
  int latest_history_frame_id_;
  int closest_history_frame_id_;
  int SClatest_history_frame_id_;
  int SCclosest_history_frame_id_;
  int latestFrameIDLoopCloure;
  float yawDiffRad;
  pcl::PointCloud<pcl::PointXYZI>::Ptr latest_keyframe_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr near_history_keyframes_;
  std::deque<pcl::PointCloud<pcl::PointXYZI>::Ptr> recent_keyframes_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr SClatest_keyframe_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr SCnear_history_keyframes_;

  std::mutex mtx_;
  std::ofstream ofs;
  std::string filename;


  bool init();
  void imu_calc(ros::Time current_time);
  void odom_calc(ros::Time current_time);
  void imu_callback(const sensor_msgs::Imu::Ptr& input);
  void points_callback(const sensor_msgs::PointCloud2::ConstPtr& input);
  void odom_callback(const nav_msgs::Odometry::ConstPtr& input);
  bool saveMapCB(lidar_localizer::save_mapRequest &req, lidar_localizer::save_mapResponse &r); 
  void extractSurroundKeyframes();
  bool saveKeyframesAndFactor();
  void publishKeyposesAndFrames();


  void performLoopClosure();
  bool detectLoopClosure();
  void correctPoses();
  

  pcl::PointCloud<pcl::PointXYZI>::Ptr transformPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud_in, const PointTPose &trans)
  {
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
                                                .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
  }

};
