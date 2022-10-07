#include "ndt_mapping.h"

ndt_mapping::ndt_mapping(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  // Default values
  nh_.param("use_imu", use_imu_, false);
  nh_.param("use_odom", use_odom_, false);
  nh_.param("scan_rate", scan_rate_, 10.0); 
  nh_.param("loop_closure_enabled", loop_closure_enabled_, false);
  nh_.param("max_iters", max_iters_, 30);
  nh_.param("step_size", step_size_, 0.1);
  nh_.param("ndt_res", ndt_res_, 0.8);
  nh_.param("trans_eps", trans_eps_, 0.01);
  nh_.param("voxel_leaf_size", voxel_leaf_size_, 0.6);
  nh_.param("min_scan_range", min_scan_range_, 1.0);
  nh_.param("max_scan_range", max_scan_range_, 130.0);
  nh_.param("incremental_voxel_update", incremental_voxel_update_, true);
  nh_.param("min_add_scan_shift",min_add_scan_shift_ ,1.0);
  nh_.param("min_add_angle_shift",min_add_angle_shift_ ,0.2);
  nh_.param("surround_search_num", surround_search_num_, 50);
  nh_.param("history_search_radius",history_search_radius_ ,10.0);
  nh_.param("history_search_num",history_search_num_ , 20);
  nh_.param("history_fitness_score", history_fitness_score_, 0.3 );
  nh_.param("ds_history_size",ds_history_size_ , 1.0);
  nh_.param<std::string>("save_dir", save_dir_, "/home/");
  nh_.param("savePCD", savePCD_, true);

  ros::NodeHandle pnh_("~");
  pnh_.param<std::string>("robot_frame", robot_frame_, std::string("base_link"));
  pnh_.param<std::string>("map_frame", map_frame_, std::string("map"));


  std::cout << "use_imu: " << use_imu_ << std::endl;
  std::cout << "use_odom: " << use_odom_ << std::endl;
  std::cout << "incremental_voxel_update: " << incremental_voxel_update_ << std::endl;
  std::cout << "loop_closure_enabled: " << loop_closure_enabled_ << std::endl;
  std::cout << "voxel_leaf_size: " << voxel_leaf_size_ << std::endl;
  std::cout << "surround_search_num: " << surround_search_num_ << std::endl;
  std::cout << "min_add_scan_shift: " << min_add_scan_shift_ << std::endl;
  std::cout << "min_add_angle_shift: " << min_add_angle_shift_ << std::endl;
  std::cout << "min_scan_range: " << min_scan_range_ << std::endl;
  std::cout << "max_scan_range: " << max_scan_range_ << std::endl;
  std::cout << "trans_eps: " <<  trans_eps_<< std::endl;
  std::cout << "step_size: " << step_size_ << std::endl;
  std::cout << "ndt_res: " << ndt_res_ << std::endl;
  std::cout << "max_iters: " <<  max_iters_<< std::endl;
  std::cout << "history_search_radius: " <<  history_search_radius_<< std::endl;
  std::cout << "history_search_num: " << history_search_num_ << std::endl;
  std::cout << "history_fitness_score: " <<  history_fitness_score_<< std::endl;
  std::cout << "ds_history_size: " <<ds_history_size_  << std::endl;


  if (nh_.getParam("tf_x", _tf_x) == false)
  {
    std::cout << "tf_x is not set." << std::endl;
  }
  if (nh_.getParam("tf_y", _tf_y) == false)
  {
    std::cout << "tf_y is not set." << std::endl;
  }
  if (nh_.getParam("tf_z", _tf_z) == false)
  {
    std::cout << "tf_z is not set." << std::endl;
  }
  if (nh_.getParam("tf_roll", _tf_roll) == false)
  {
    std::cout << "tf_roll is not set." << std::endl;
  }
  if (nh_.getParam("tf_pitch", _tf_pitch) == false)
  {
    std::cout << "tf_pitch is not set." << std::endl;
  }
  if (nh_.getParam("tf_yaw", _tf_yaw) == false)
  {
    std::cout << "tf_yaw is not set." << std::endl;
  }
  std::cout << "(tf_x,tf_y,tf_z,tf_roll,tf_pitch,tf_yaw): (" << _tf_x << ", " << _tf_y << ", " << _tf_z << ", "
            << _tf_roll << ", " << _tf_pitch << ", " << _tf_yaw << ")" << std::endl;

  Eigen::Translation3f tl_btol(_tf_x, _tf_y, _tf_z);                 // tl: translation
  Eigen::AngleAxisf rot_x_btol(_tf_roll, Eigen::Vector3f::UnitX());  // rot: rotation
  Eigen::AngleAxisf rot_y_btol(_tf_pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(_tf_yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();
  tf_ltob_ = tf_btol_.inverse();

  if (!init())
  {
    exit(-1);
  }

  odom_sub_ = nh.subscribe("/odom", 100000, &ndt_mapping::odom_callback,this);
  points_sub_ = nh_.subscribe("velodyne_points", 100000, &ndt_mapping::points_callback,this); //velodyne_points
//  ndt_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  pub_laser_cloud_surround_ = nh_.advertise<sensor_msgs::PointCloud2>("/ndt_map", 1000);
  pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 1000);
  current_pose_pub_ = nh_.advertise<nav_msgs::Odometry>("/current_pose", 1000);
  srv_save_map_ = nh_.advertiseService("/save_map", &ndt_mapping::saveMapCB, this);


 // pub_history_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/history_keyframes", 1000);
  pub_recent_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/recent_keyframes", 1000);
  //pub_icp_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/icp_keyframes", 1000);

}; 

//ndt_mapping::~ndt_mapping(){}; 
bool ndt_mapping::init()
{
  map_.header.frame_id = map_frame_;

  ISAM2Params params;
  params.relinearizeThreshold = 0.01;
  params.relinearizeSkip = 1;
  isam = new ISAM2(params);
  gtsam::Vector vector6(6);
  gtsam::Vector Vector6_prior(6);
  gtsam::Vector Vector6_odom(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  //Vector6_prior << 1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8;
  //Vector6_odom << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);

  scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  map_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  filtered_scan_ptr.reset(new pcl::PointCloud<pcl::PointXYZI>());
  current_pose_.x = current_pose_.y = current_pose_.z = 0.0;current_pose_.roll = current_pose_.pitch = current_pose_.yaw = 0.0;
  previous_pose_.x = previous_pose_.y = previous_pose_.z = 0.0;previous_pose_.roll = previous_pose_.pitch = previous_pose_.yaw = 0.0;
  offset_odom_x = offset_odom_y = offset_odom_z = offset_odom_roll = offset_odom_pitch = offset_odom_yaw = 0.0 ;
  guess_pose.x = guess_pose.y = guess_pose.z = 0.0;guess_pose.roll = guess_pose.pitch = guess_pose.yaw = 0.0;
  added_pose.x = added_pose.y = added_pose.z = 0.0;added_pose.roll = added_pose.pitch = added_pose.yaw = 0.0;

  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);  
  ds_source_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  ds_history_keyframes_.setLeafSize(ds_history_size_, ds_history_size_, ds_history_size_);

  cloud_keyposes_3d_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointTPose>());

  latest_keyframe_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  near_history_keyframes_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  SClatest_keyframe_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  SCnear_history_keyframes_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  kdtree_poses_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  ndt.setTransformationEpsilon(trans_eps_);
  ndt.setStepSize(step_size_);
  ndt.setResolution(ndt_res_);
  ndt.setMaximumIterations(max_iters_);

  is_first_map_ = true;
  initial_scan_loaded = 0;
  loop_closed_ = false;

  ROS_INFO("init.");
  return true;
}

void ndt_mapping::odom_callback(const nav_msgs::Odometry::ConstPtr& input)
{
  // std::cout << __func__ << std::endl;

  odom = *input;
  odom_calc(input->header.stamp);
}

void ndt_mapping::odom_calc(ros::Time current_time)
{
  static ros::Time previous_time = current_time;
  double diff_time = (current_time - previous_time).toSec();

  double diff_odom_roll = odom.twist.twist.angular.x * diff_time;
  double diff_odom_pitch = odom.twist.twist.angular.y * diff_time;
  double diff_odom_yaw = odom.twist.twist.angular.z * diff_time;

  current_pose_odom.roll += diff_odom_roll;
  current_pose_odom.pitch += diff_odom_pitch;
  current_pose_odom.yaw += diff_odom_yaw;

  double diff_distance = odom.twist.twist.linear.x * diff_time;
  offset_odom_x += diff_distance * cos(-current_pose_odom.pitch) * cos(current_pose_odom.yaw);
  offset_odom_y += diff_distance * cos(-current_pose_odom.pitch) * sin(current_pose_odom.yaw);
  offset_odom_z += diff_distance * sin(-current_pose_odom.pitch);

  offset_odom_roll += diff_odom_roll;
  offset_odom_pitch += diff_odom_pitch;
  offset_odom_yaw += diff_odom_yaw;

  guess_pose_odom.x = previous_pose_.x + offset_odom_x;
  guess_pose_odom.y = previous_pose_.y + offset_odom_y;
  guess_pose_odom.z = previous_pose_.z + offset_odom_z;
  guess_pose_odom.roll = previous_pose_.roll + offset_odom_roll;
  guess_pose_odom.pitch = previous_pose_.pitch + offset_odom_pitch;
  guess_pose_odom.yaw = previous_pose_.yaw + offset_odom_yaw;

  previous_time = current_time;
}

void ndt_mapping::points_callback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
  std::lock_guard<std::mutex> lock(mtx_);
  auto start = std::chrono::system_clock::now();
 
 extractSurroundKeyframes();

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
 
  tf::Quaternion q;

  Eigen::Matrix4f t_localizer(Eigen::Matrix4f::Identity());
  Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());
  //static tf::TransformBroadcaster br;
  tf::Transform transform;

  current_scan_time = input->header.stamp;

  scan_ptr->clear();
  pcl::fromROSMsg(*input, *tmp_cloud);

// trans to baselink_frame
  pcl::transformPointCloud(*tmp_cloud, *scan_ptr, tf_ltob_); //tf_btol_ tf_ltob_
  tmp_cloud->clear();
//downsample
  voxel_grid_filter_.setInputCloud(scan_ptr);
  voxel_grid_filter_.filter(*tmp_cloud);
  scan_ptr->clear();
//filter by distance
  double r;
  for (const auto &p : tmp_cloud->points)
  {
    r = sqrt(p.x * p.x + p.y * p.y);
    if (r > min_scan_range_ && r < max_scan_range_)
    {
      scan_ptr->points.push_back(p);
    }
  }

  // Add initial point cloud to velodyne_map
  if (initial_scan_loaded == 0)
  {
    *map_ptr += *scan_ptr;
    initial_scan_loaded = 1;
  }

  ndt.setInputSource(scan_ptr);

  if (is_first_map_ == true){
    ndt.setInputTarget(map_ptr);
    is_first_map_ = false;
  }
  
  if (use_imu_ == false && use_odom_ == true)
    odom_calc(current_scan_time);

  if (use_imu_ == false && use_odom_ == true)
    guess_pose_for_ndt = guess_pose_odom;
  else
    guess_pose_for_ndt = current_pose_;

  Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

  Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);
  Eigen::Matrix4f init_guess =
      (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
 
  ROS_INFO("start ndt align");

  ndt.align(*output_cloud, init_guess);
  t_localizer = ndt.getFinalTransformation();

  t_base_link = t_localizer; // * tf_ltob_;

 

  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                 static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                 static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                 static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                 static_cast<double>(t_base_link(2, 2)));

  // Update current_pose_.
  current_pose_.x = t_base_link(0, 3);current_pose_.y = t_base_link(1, 3);current_pose_.z = t_base_link(2, 3);
  mat_b.getRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw, 1);//mat2rpy

  transform.setOrigin(tf::Vector3(current_pose_.x, current_pose_.y, current_pose_.z));
  q.setRPY(current_pose_.roll, current_pose_.pitch, current_pose_.yaw); //q from rpy
  transform.setRotation(q);//trans from q

  br_.sendTransform(tf::StampedTransform(transform, input->header.stamp, map_frame_, robot_frame_));

  current_pose_odom.x = current_pose_.x; current_pose_odom.y = current_pose_.y; current_pose_odom.z = current_pose_.z;
  current_pose_odom.roll = current_pose_.roll; current_pose_odom.pitch = current_pose_.pitch; current_pose_odom.yaw = current_pose_.yaw;

  previous_pose_.x = current_pose_.x;previous_pose_.y = current_pose_.y;previous_pose_.z = current_pose_.z;
  previous_pose_.roll = current_pose_.roll;previous_pose_.pitch = current_pose_.pitch;previous_pose_.yaw = current_pose_.yaw;
  
  offset_odom_x = offset_odom_y = offset_odom_z = offset_odom_roll = offset_odom_pitch = offset_odom_yaw = 0.0 ;

  double shift = sqrt(pow(current_pose_.x - added_pose.x, 2.0) + pow(current_pose_.y - added_pose.y, 2.0));

  if (shift >= min_add_scan_shift_)
  {
    if (saveKeyframesAndFactor())
    {
      std::cout<<"shift >= min_add_scan_shift_ && saveKeyframesAndFactor == true "<<std::endl;
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::transformPointCloud(*scan_ptr, *transformed_scan_ptr, t_localizer);
      *map_ptr += *transformed_scan_ptr;
       added_pose.x = current_pose_.x;added_pose.y = current_pose_.y;added_pose.z = current_pose_.z;
       added_pose.roll = current_pose_.roll;added_pose.pitch = current_pose_.pitch;added_pose.yaw = current_pose_.yaw;
       ndt.setInputTarget(map_ptr);
    }
  }

  //sensor_msgs::PointCloud2::Ptr map_msg_ptr(new sensor_msgs::PointCloud2);
  //pcl::toROSMsg(*map_ptr, *map_msg_ptr);
  //ndt_map_pub_.publish(*map_msg_ptr);// it makes rviz very slow.

  current_pose_msg_.header.frame_id = "map";
  current_pose_msg_.child_frame_id = "base_link";
  current_pose_msg_.header.stamp = input->header.stamp;
  current_pose_msg_.pose.pose.position.x = current_pose_.x;
  current_pose_msg_.pose.pose.position.y = current_pose_.y;
  current_pose_msg_.pose.pose.position.z = current_pose_.z;
  current_pose_msg_.pose.pose.orientation.x = q.x();
  current_pose_msg_.pose.pose.orientation.y = q.y();
  current_pose_msg_.pose.pose.orientation.z = q.z();
  current_pose_msg_.pose.pose.orientation.w = q.w();

  current_pose_pub_.publish(current_pose_msg_);

  namefilepose.open("./ndt_pose.txt",std::ios::app);
  namefilepose<<input->header.stamp<<" ";
        
  namefilepose<< current_pose_msg_.pose.pose.position.x <<" ";
  namefilepose<< current_pose_msg_.pose.pose.position.y <<" ";
  namefilepose<< current_pose_msg_.pose.pose.position.z <<" ";
  namefilepose<< current_pose_msg_.pose.pose.orientation.x<<" ";
  namefilepose<< current_pose_msg_.pose.pose.orientation.y<<" ";
  namefilepose<< current_pose_msg_.pose.pose.orientation.z<<" ";
  namefilepose<< current_pose_msg_.pose.pose.orientation.w;
  namefilepose<<std::endl;
  namefilepose.close();

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << input->header.seq << std::endl;
  std::cout << "Number of scan points: " << scan_ptr->size() << " points." << std::endl;
 // std::cout << "Number of filtered scan points: " << filtered_scan_ptr->size() << " points." << std::endl;
 // std::cout << "transformed_scan_ptr: " << transformed_scan_ptr->points.size() << " points." << std::endl;
  std::cout << "map: " << map_ptr->size() << " points." << std::endl;
  std::cout << "NDT has converged: " << ndt.hasConverged() << std::endl;
  std::cout << "Fitness score: " << ndt.getFitnessScore() << std::endl;
  std::cout << "Number of iteration: " << ndt.getFinalNumIteration() << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << ", " << current_pose_.roll
            << ", " << current_pose_.pitch << ", " << current_pose_.yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_localizer << std::endl;
  std::cout << "shift: " << shift << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  correctPoses(); //for loop closure
}

void ndt_mapping::run() {}

void ndt_mapping::visualThread()
{
  ros::Duration duration(2.5);
  while (ros::ok())
  {
    publishKeyposesAndFrames();
    duration.sleep();
  }

  lidar_localizer::save_mapRequest req;
  lidar_localizer::save_mapResponse res;

  if(!saveMapCB(req,res))
  {
    ROS_WARN("fail to save map");
  }
}

void ndt_mapping::publishKeyposesAndFrames()
{
  if (pub_keyposes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }
  if (pub_laser_cloud_surround_.getNumSubscribers() > 0)
  {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_keypose(new pcl::PointCloud<pcl::PointXYZI>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    for (int i = 0; i < cloud_keyframes_.size(); ++i)
    {
      pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *map_keypose += *tmp;
      num_points += tmp->points.size();
    }
    pcl::toROSMsg(*map_keypose, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_laser_cloud_surround_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }
}


void ndt_mapping::extractSurroundKeyframes()
{
  if (cloud_keyframes_.empty()) //cloudKeyPoses3D->points.empty() == true
  {
    return;
  }

  bool target_updated = false;
  if (loop_closure_enabled_)
  {
    if (recent_keyframes_.size() < surround_search_num_)
    {
      recent_keyframes_.clear();
      for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i)
      {
        int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
        pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
        recent_keyframes_.push_back(tf_cloud);
        if (recent_keyframes_.size() >= surround_search_num_)
        {
          break;
        }
      }
      target_updated = true;
    }
    else
    {
      static int latest_frame_id = cloud_keyframes_.size() - 1;
      if (latest_frame_id != cloud_keyframes_.size() - 1)
      {
        latest_frame_id = cloud_keyframes_.size() - 1;
        recent_keyframes_.pop_back();
        pcl::PointCloud<pcl::PointXYZI>::Ptr tf_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id], cloud_keyposes_6d_->points[latest_frame_id]);
        recent_keyframes_.push_front(tf_cloud);
        target_updated = true;
      }
    }
  }
 // ROS_WARN("target_updated");
  if (target_updated)
  {
    map_ptr->clear(); 
    for (auto keyframe : recent_keyframes_)
    {
      //std::cout << "Number of recent_keyframes_ points: " << keyframe->size() << " points." << std::endl;
      *map_ptr += *keyframe;
    }
    ndt.setInputTarget(map_ptr);
   // ROS_INFO("new ndt target set");
  }

  if (pub_recent_keyframes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*map_ptr, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_recent_keyframes_.publish(msg);
  }
}

bool ndt_mapping::saveKeyframesAndFactor()
{
  // update (cur_pose_ndt_)
  double roll, pitch, yaw;
  roll = current_pose_.roll;
  pitch = current_pose_.pitch;
  yaw = current_pose_.yaw;
 // tf::Matrix3x3(tf::Quaternion(cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z, cur_pose_ndt_.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
  if (cloud_keyposes_3d_->points.empty())
  {
    std::cout<<"cloud_keyposes_3d_->points.empty()"<<std::endl;
    // gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(current_pose_.x, current_pose_.y, current_pose_.z)), prior_noise_));
    initial_estimate_.insert(0, Pose3(Rot3::RzRyRx(current_pose_.roll, current_pose_.pitch, current_pose_.yaw), Point3(current_pose_.x, current_pose_.y,current_pose_.z)));
    pre_keypose = current_pose_;
  }
  else
  {
    const auto &pre_pose_ = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];
    double keyframedist = sqrt(pow(current_pose_.x - pre_pose_.x, 2) + pow(current_pose_.y - pre_pose_.y, 2) + pow(pre_pose_.z - pre_pose_.z, 2));
            
    if ((keyframedist < min_add_scan_shift_)&&(abs(roll)<min_add_angle_shift_)&&(abs(pitch)<min_add_angle_shift_)&&(abs(yaw)<min_add_angle_shift_))
    {     
      return false;
    }
   
    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pre_pose_.roll, pre_pose_.pitch, pre_pose_.yaw), Point3(pre_pose_.x, pre_pose_.y, pre_pose_.z));
    gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(current_pose_.x, current_pose_.y, current_pose_.z));
    //gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(current_pose_.x, current_pose_.y, current_pose_.z * 0));
    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(), pose_from.between(pose_to), odom_noise_));
    //initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(current_pose_.x, current_pose_.y, current_pose_.z * 0)));
   initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(current_pose_.x, current_pose_.y, current_pose_.z)));
  }

  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  pcl::PointXYZI this_pose_3d;
  PointXYZIRPYT this_pose_6d;
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);

  this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();
  this_pose_6d.roll = latest_estimate.rotation().roll();
  this_pose_6d.pitch = latest_estimate.rotation().pitch();
  this_pose_6d.yaw = latest_estimate.rotation().yaw();
  this_pose_6d.time = current_scan_time.toSec();
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  // std::cout << "pre_keypose: (" << pre_keypose_.pose.pose.position.x << ", " << pre_keypose_.pose.pose.position.y << ", " << pre_keypose_.pose.pose.position.z << "; " << std::endl;
  std::cout << "current_pose: (" << current_pose_.x << ", " << current_pose_.y << ", " << current_pose_.z << "; "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "this_pose_6d: (" << this_pose_6d.x << ", " << this_pose_6d.y << ", " << this_pose_6d.z << "; "
            << this_pose_6d.roll << ", " << this_pose_6d.pitch << ", " << this_pose_6d.yaw << ")" << std::endl;

  if (cloud_keyposes_3d_->points.size() > 1)
  {
    pre_keypose.x = this_pose_3d.x;
    pre_keypose.y = this_pose_3d.y;
    pre_keypose.z = this_pose_3d.z;
    pre_keypose.roll = latest_estimate.rotation().roll();
    pre_keypose.pitch = latest_estimate.rotation().pitch();
    pre_keypose.yaw = latest_estimate.rotation().yaw();
  }

    current_pose_=pre_keypose;
//save key frame
  pcl::PointCloud<pcl::PointXYZI>::Ptr cur_keyframe(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*scan_ptr , *cur_keyframe);  //scan_ptr
  for (auto &p : cur_keyframe->points)
  {
    p.intensity = this_pose_3d.intensity;
  }

  /* 
    Scan Context loop detector 
  */
   pcl::PointCloud<pcl::PointXYZI>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<pcl::PointXYZI>());
   pcl::copyPointCloud(*scan_ptr,  *thisRawCloudKeyFrame);
   scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);


  cloud_keyframes_.push_back(cur_keyframe);


  ROS_WARN("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  return true;
}


bool ndt_mapping::saveMapCB(lidar_localizer::save_mapRequest &req, lidar_localizer::save_mapResponse &res)
{
  std::string saveMapDirectory;

  std::cout << "****************************************************" << std::endl;
  std::cout << "Saving map to pcd files ..." << std::endl;
  if(req.destination.empty()) 
     saveMapDirectory = std::getenv("HOME") + save_dir_;
  else saveMapDirectory = std::getenv("HOME") + req.destination;

  std::cout << "Save destination: " << saveMapDirectory << std::endl;
  // create directory and remove old files;
  int unused = system((std::string("exec rm -r ") + saveMapDirectory).c_str());
  unused = system((std::string("mkdir -p ") + saveMapDirectory).c_str());

  int num_points = 0;
  pcl::PointCloud<pcl::PointXYZI>::Ptr mapPCD(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_filteredPCD(new pcl::PointCloud<pcl::PointXYZI>());

  for (int i = 0; i < cloud_keyframes_.size(); ++i)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmpPCD(new pcl::PointCloud<pcl::PointXYZI>());
    tmpPCD = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *mapPCD += *tmpPCD;
    num_points += tmpPCD->points.size();
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr posesPCD(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *posesPCD);
  posesPCD->width = posesPCD->points.size();
  posesPCD->height = 1;
  posesPCD->is_dense = false;
  pcl::io::savePCDFile(saveMapDirectory + "/pose.pcd", *posesPCD);


  mapPCD->width = mapPCD->points.size();
  mapPCD->height = 1;
  mapPCD->is_dense = false;
 // pcl::io::savePCDFile(saveMapDirectory + "/map.pcd", *map);

  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter;
  voxel_grid_filter.setLeafSize(0.2, 0.2, 0.2);
  voxel_grid_filter.setInputCloud(mapPCD);
  voxel_grid_filter.filter(*map_filteredPCD);
  pcl::io::savePCDFile(saveMapDirectory + "/filtered_map.pcd",  *map_filteredPCD);

  int ret = pcl::io::savePCDFileASCII(saveMapDirectory + "/mapascii.pcd", *mapPCD);
  
  res.success = ret == 0;

  std::cout << "****************************************************" << std::endl;
  std::cout << "Saving map to pcd files completed\n" << std::endl;

  return true;
}

void ndt_mapping::correctPoses()
{
  if (loop_closed_)
  {
    recent_keyframes_.clear();
    ROS_WARN("correctPoses");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i)
    {
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x = isam_current_estimate_.at<Pose3>(i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y = isam_current_estimate_.at<Pose3>(i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z = isam_current_estimate_.at<Pose3>(i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }

    loop_closed_ = false;
  }
}

void ndt_mapping::loopClosureThread()
{
  if (!loop_closure_enabled_)
  {
    return;
  }
  //ros::Duration duration(1);
  ros::Rate rate(1.0);
  while (ros::ok())
  {
    performLoopClosure();
   // duration.sleep();
  }
}

void ndt_mapping::performLoopClosure()
{
  if (cloud_keyposes_3d_->points.empty())
  {
    return;
  }

  if (!detectLoopClosure())
  {
    return;
  }
  else
  {
    ROS_WARN("detected loop closure");
  }

  auto start = std::chrono::system_clock::now();

    bool isValidRSloopFactor = false;
    bool isValidSCloopFactor = false;
  /*
   * 1. RS loop factor (radius search)
   */
  if(closest_history_frame_id_!= -1 ) {
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(latest_keyframe_);
  icp.setInputTarget(near_history_keyframes_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
                                        .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if( (has_converged == false )|| (fitness_score > history_fitness_score_))
  {
    isValidRSloopFactor = false;
    return;
  }
  else {
      std::cout << "[RS] The detected loop factor is added between Current [ " << latest_history_frame_id_ << " ] and RS nearest [ " << closest_history_frame_id_ << " ]" << std::endl;
      isValidRSloopFactor = true;
  }

  if( isValidRSloopFactor == true ) {
    ROS_WARN("RSloop closed");

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(cloud_keyposes_6d_->points[closest_history_frame_id_].roll, cloud_keyposes_6d_->points[closest_history_frame_id_].pitch, cloud_keyposes_6d_->points[closest_history_frame_id_].yaw),
                               Point3(cloud_keyposes_6d_->points[closest_history_frame_id_].x, cloud_keyposes_6d_->points[closest_history_frame_id_].y, cloud_keyposes_6d_->points[closest_history_frame_id_].z));
  float noise_score = fitness_score; //0.5
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);
  robustNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(vector6)
  ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)


  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to), constraint_noise_)); //robustNoiseModel
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);
  }
}
  /*
   * 2. SC loop factor (scan context)
   */

  if(SCclosest_history_frame_id_!= -1 ) {
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(SClatest_keyframe_);
  icp.setInputTarget(SCnear_history_keyframes_);
  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(cloud_keyposes_6d_->points[SClatest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[SClatest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[SClatest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
                                        .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[SClatest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[SClatest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[SClatest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if( (has_converged == false )|| (fitness_score > history_fitness_score_))
  {
    isValidSCloopFactor = false;
    return;
  }
  else {
      std::cout << "[SC] The detected loop factor is added between Current [ " << SClatest_history_frame_id_ << " ] and RS nearest [ " << SCclosest_history_frame_id_ << " ]" << std::endl;
      isValidSCloopFactor = true;
  }

  if( isValidSCloopFactor == true ) {
    ROS_WARN("SCloop closed");

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));
  float noise_score = fitness_score; //0.5
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);
  robustNoiseModel = gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
        gtsam::noiseModel::Diagonal::Variances(vector6)
  ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)


  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(SClatest_history_frame_id_, SCclosest_history_frame_id_, pose_from.between(pose_to), constraint_noise_)); //robustNoiseModel
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);
  }
}

  loop_closed_ = true;
}

bool ndt_mapping::detectLoopClosure()
{
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  pcl::PointXYZI cur_pose;
  cur_pose.x = current_pose_.x;
  cur_pose.y = current_pose_.y;
  cur_pose.z = current_pose_.z;
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);      

  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i)
  {
    if (abs(current_scan_time.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time) > 30.)
    {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  // time too short
  if (closest_history_frame_id_ == -1)
  {
    //return false;
  }
  else{ //if (closest_history_frame_id_ != -1)
  pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[latest_history_frame_id_], cloud_keyposes_6d_->points[latest_history_frame_id_]), *latest_keyframe_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
  {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_)
    {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  ds_history_keyframes_.setInputCloud(tmp_cloud);
  ds_history_keyframes_.filter(*near_history_keyframes_);
  }
  /* 
   * 2. Scan context-based global localization 
   */
  SClatest_keyframe_->clear();
  SCnear_history_keyframes_->clear();
    
  SClatest_history_frame_id_ = cloud_keyframes_.size() - 1;
  SCclosest_history_frame_id_ = -1; 
  auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
  SCclosest_history_frame_id_ = detectResult.first;
  yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)
  // if all close, reject
  if (SCclosest_history_frame_id_ == -1){ //if ((closest_history_frame_id_ == -1)&&(SCclosest_history_frame_id_ == -1))
      return false;
   }

  pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[SClatest_history_frame_id_], cloud_keyposes_6d_->points[SClatest_history_frame_id_]), *SClatest_keyframe_);

  pcl::PointCloud<pcl::PointXYZI>::Ptr SCtmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
  {
    j = SCclosest_history_frame_id_ + i;
    if (j < 0 || j >= SClatest_history_frame_id_)
    {
      continue;
    }
    *SCtmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
    }

    ds_history_keyframes_.setInputCloud(SCtmp_cloud);
    ds_history_keyframes_.filter(*SCnear_history_keyframes_);

  return true;
}