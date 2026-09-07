#include <lidar_localization/lidar_localization_component.hpp>
#include <algorithm>
#include <cmath>

namespace
{
double stampSeconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

double quaternionAngle(const Eigen::Quaterniond & lhs, const Eigen::Quaterniond & rhs)
{
  const Eigen::Quaterniond delta = lhs.conjugate() * rhs;
  return std::abs(2.0 * std::atan2(delta.vec().norm(), std::abs(delta.w())));
}
}

PCLLocalization::PCLLocalization(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("lidar_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  tflistener_(tfbuffer_),
  broadcaster_(this)
{
  declare_parameter("global_frame_id", "map");
  declare_parameter("odom_frame_id", "odom");
  declare_parameter("base_frame_id", "base_link");
  declare_parameter("registration_method", "NDT");
  declare_parameter("score_threshold", 2.0);
  declare_parameter("ndt_resolution", 1.0);
  declare_parameter("ndt_step_size", 0.1);
  declare_parameter("transform_epsilon", 0.01);
  declare_parameter("voxel_leaf_size", 0.2);
  declare_parameter("scan_max_range", 100.0);
  declare_parameter("scan_min_range", 1.0);
  declare_parameter("scan_period", 0.1);
  declare_parameter("use_pcd_map", false);
  declare_parameter("map_path", "/map/map.pcd");
  declare_parameter("set_initial_pose", false);
  declare_parameter("initial_pose_x", 0.0);
  declare_parameter("initial_pose_y", 0.0);
  declare_parameter("initial_pose_z", 0.0);
  declare_parameter("initial_pose_qx", 0.0);
  declare_parameter("initial_pose_qy", 0.0);
  declare_parameter("initial_pose_qz", 0.0);
  declare_parameter("initial_pose_qw", 1.0);
  declare_parameter("use_odom", false);
  declare_parameter("use_imu", false);
  declare_parameter("enable_debug", false);
  declare_parameter("ndt_num_threads", 0);
  declare_parameter("ndt_max_iterations", 50);
  declare_parameter("far_point_boost_min_range", 0.0);
  declare_parameter("far_point_boost_factor", 1);
  declare_parameter("enable_map_odom_tf", false);
  declare_parameter("enable_timer_publishing", false);
  declare_parameter("pose_publish_frequency", 20.0);
  declare_parameter("lock_planar", false);
  declare_parameter("max_pose_jump_translation", -1.0);
  declare_parameter("max_pose_jump_rotation", -1.0);
  declare_parameter("max_map_odom_update_translation", -1.0);
  declare_parameter("max_map_odom_update_rotation", -1.0);
  declare_parameter("ndt_align_interval_s", 0.0);
  declare_parameter("map_odom_smoothing", 1.0);
}

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturn PCLLocalization::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  initializeParameters();
  initializePubSub();
  initializeRegistration();

  path_ptr_ = std::make_shared<nav_msgs::msg::Path>();
  path_ptr_->header.frame_id = global_frame_id_;

  RCLCPP_INFO(get_logger(), "Configuring end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Activating");

  pose_pub_->on_activate();
  path_pub_->on_activate();
  initial_map_pub_->on_activate();
  ndt_aligned_scan_pub_->on_activate();

  if (set_initial_pose_) {
    auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

    msg->header.stamp = now();
    msg->header.frame_id = global_frame_id_;
    msg->pose.pose.position.x = initial_pose_x_;
    msg->pose.pose.position.y = initial_pose_y_;
    msg->pose.pose.position.z = initial_pose_z_;
    msg->pose.pose.orientation.x = initial_pose_qx_;
    msg->pose.pose.orientation.y = initial_pose_qy_;
    msg->pose.pose.orientation.z = initial_pose_qz_;
    msg->pose.pose.orientation.w = initial_pose_qw_;

    geometry_msgs::msg::PoseStamped::SharedPtr pose_stamped(new geometry_msgs::msg::PoseStamped);
    pose_stamped->header.stamp = msg->header.stamp;
    pose_stamped->header.frame_id = global_frame_id_;
    pose_stamped->pose = msg->pose.pose;
    path_ptr_->poses.push_back(*pose_stamped);

    initialPoseReceived(msg);
  }

  if (use_pcd_map_) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile(map_path_, *map_cloud_ptr);
    RCLCPP_INFO(get_logger(), "Map Size %ld", map_cloud_ptr->size());

    sensor_msgs::msg::PointCloud2::SharedPtr map_msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*map_cloud_ptr, *map_msg_ptr);
    map_msg_ptr->header.frame_id = global_frame_id_;
    initial_map_pub_->publish(*map_msg_ptr);
    RCLCPP_INFO(get_logger(), "Initial Map Published");

    if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
      pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
      voxel_grid_filter_.setInputCloud(map_cloud_ptr);
      voxel_grid_filter_.filter(*filtered_cloud_ptr);
      registration_->setInputTarget(filtered_cloud_ptr);
    } else {
      registration_->setInputTarget(map_cloud_ptr);
    }

    map_recieved_ = true;
  }

  RCLCPP_INFO(get_logger(), "Activating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  pose_pub_->on_deactivate();
  path_pub_->on_deactivate();
  initial_map_pub_->on_deactivate();
  ndt_aligned_scan_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivating end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Cleaning Up");
  initial_pose_sub_.reset();
  initial_map_pub_.reset();
  ndt_aligned_scan_pub_.reset();
  path_pub_.reset();
  pose_pub_.reset();
  odom_sub_.reset();
  cloud_sub_.reset();
  imu_sub_.reset();

  RCLCPP_INFO(get_logger(), "Cleaning Up end");
  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_shutdown(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(get_logger(), "Shutting Down from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

CallbackReturn PCLLocalization::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_FATAL(get_logger(), "Error Processing from %s", state.label().c_str());

  return CallbackReturn::SUCCESS;
}

void PCLLocalization::initializeParameters()
{
  RCLCPP_INFO(get_logger(), "initializeParameters");
  get_parameter("global_frame_id", global_frame_id_);
  get_parameter("odom_frame_id", odom_frame_id_);
  get_parameter("base_frame_id", base_frame_id_);
  get_parameter("registration_method", registration_method_);
  get_parameter("score_threshold", score_threshold_);
  get_parameter("ndt_resolution", ndt_resolution_);
  get_parameter("ndt_step_size", ndt_step_size_);
  get_parameter("ndt_num_threads", ndt_num_threads_);
  get_parameter("transform_epsilon", transform_epsilon_);
  get_parameter("voxel_leaf_size", voxel_leaf_size_);
  get_parameter("scan_max_range", scan_max_range_);
  get_parameter("scan_min_range", scan_min_range_);
  get_parameter("scan_period", scan_period_);
  get_parameter("use_pcd_map", use_pcd_map_);
  get_parameter("map_path", map_path_);
  get_parameter("set_initial_pose", set_initial_pose_);
  get_parameter("initial_pose_x", initial_pose_x_);
  get_parameter("initial_pose_y", initial_pose_y_);
  get_parameter("initial_pose_z", initial_pose_z_);
  get_parameter("initial_pose_qx", initial_pose_qx_);
  get_parameter("initial_pose_qy", initial_pose_qy_);
  get_parameter("initial_pose_qz", initial_pose_qz_);
  get_parameter("initial_pose_qw", initial_pose_qw_);
  get_parameter("use_odom", use_odom_);
  get_parameter("use_imu", use_imu_);
  get_parameter("enable_debug", enable_debug_);
  get_parameter("ndt_max_iterations", ndt_max_iterations_);
  get_parameter("far_point_boost_min_range", far_point_boost_min_range_);
  get_parameter("far_point_boost_factor", far_point_boost_factor_);
  get_parameter("enable_map_odom_tf", enable_map_odom_tf_);
  get_parameter("enable_timer_publishing", enable_timer_publishing_);
  get_parameter("pose_publish_frequency", pose_publish_frequency_);
  get_parameter("lock_planar", lock_planar_);
  get_parameter("max_pose_jump_translation", max_pose_jump_translation_);
  get_parameter("max_pose_jump_rotation", max_pose_jump_rotation_);
  get_parameter("max_map_odom_update_translation", max_map_odom_update_translation_);
  get_parameter("max_map_odom_update_rotation", max_map_odom_update_rotation_);
  get_parameter("ndt_align_interval_s", ndt_align_interval_s_);
  get_parameter("map_odom_smoothing", map_odom_smoothing_);
  RCLCPP_INFO(get_logger(),"global_frame_id: %s", global_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"base_frame_id: %s", base_frame_id_.c_str());
  RCLCPP_INFO(get_logger(),"registration_method: %s", registration_method_.c_str());
  RCLCPP_INFO(get_logger(),"ndt_resolution: %lf", ndt_resolution_);
  RCLCPP_INFO(get_logger(),"ndt_step_size: %lf", ndt_step_size_);
  RCLCPP_INFO(get_logger(),"ndt_num_threads: %d", ndt_num_threads_);
  RCLCPP_INFO(get_logger(),"transform_epsilon: %lf", transform_epsilon_);
  RCLCPP_INFO(get_logger(),"voxel_leaf_size: %lf", voxel_leaf_size_);
  RCLCPP_INFO(get_logger(),"scan_max_range: %lf", scan_max_range_);
  RCLCPP_INFO(get_logger(),"scan_min_range: %lf", scan_min_range_);
  RCLCPP_INFO(get_logger(),"scan_period: %lf", scan_period_);
  RCLCPP_INFO(get_logger(),"use_pcd_map: %d", use_pcd_map_);
  RCLCPP_INFO(get_logger(),"map_path: %s", map_path_.c_str());
  RCLCPP_INFO(get_logger(),"set_initial_pose: %d", set_initial_pose_);
  RCLCPP_INFO(get_logger(),"use_odom: %d", use_odom_);
  RCLCPP_INFO(get_logger(),"use_imu: %d", use_imu_);
  RCLCPP_INFO(get_logger(),"enable_debug: %d", enable_debug_);
  RCLCPP_INFO(get_logger(), "NDT controls: max_iterations=%d align_interval=%.3f jump_translation=%.3f jump_rotation=%.3f map_odom_smoothing=%.2f", ndt_max_iterations_, ndt_align_interval_s_, max_pose_jump_translation_, max_pose_jump_rotation_, map_odom_smoothing_);
  RCLCPP_INFO(get_logger(), "map->odom: enabled=%d external_translation_gate=%.3f external_rotation_gate=%.3f", enable_map_odom_tf_, max_map_odom_update_translation_, max_map_odom_update_rotation_);
}

void PCLLocalization::initializePubSub()
{
  RCLCPP_INFO(get_logger(), "initializePubSub");

  pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "pcl_pose",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "path",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  initial_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "initial_map",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  ndt_aligned_scan_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "ndt_aligned_scan", rclcpp::SensorDataQoS());

  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&PCLLocalization::initialPoseReceived, this, std::placeholders::_1));

  map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&PCLLocalization::mapReceived, this, std::placeholders::_1));

  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "odom", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::odomReceived, this, std::placeholders::_1));

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    "imu", rclcpp::SensorDataQoS(),
    std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "initializePubSub end");
}

void PCLLocalization::initializeRegistration()
{
  RCLCPP_INFO(get_logger(), "initializeRegistration");

  if (registration_method_ == "GICP") {
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>> gicp(
      new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp;
  }
  else if (registration_method_ == "NDT") {
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>> ndt(
      new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt->setStepSize(ndt_step_size_);
    ndt->setResolution(ndt_resolution_);
    ndt->setTransformationEpsilon(transform_epsilon_);
    registration_ = ndt;
  }
  else if (registration_method_ == "NDT_OMP") {
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr ndt_omp(
      new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    ndt_omp->setStepSize(ndt_step_size_);
    ndt_omp->setResolution(ndt_resolution_);
    ndt_omp->setTransformationEpsilon(transform_epsilon_);
    if (ndt_num_threads_ > 0) {
      ndt_omp->setNumThreads(ndt_num_threads_);
    } else {
      ndt_omp->setNumThreads(omp_get_max_threads());
    }
    registration_ = ndt_omp;
  }
  else if (registration_method_ == "GICP_OMP") {
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr gicp_omp(
      new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    gicp_omp->setTransformationEpsilon(transform_epsilon_);
    registration_ = gicp_omp;
  }
  else {
    RCLCPP_ERROR(get_logger(), "Invalid registration method.");
    exit(EXIT_FAILURE);
  }


  voxel_grid_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  if (registration_method_ == "NDT") {
    auto ndt = boost::dynamic_pointer_cast<pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (ndt) {
      ndt->setMaximumIterations(ndt_max_iterations_);
    }
  } else if (registration_method_ == "NDT_OMP") {
    auto ndt = boost::dynamic_pointer_cast<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (ndt) {
      ndt->setMaximumIterations(ndt_max_iterations_);
    }
  }
  RCLCPP_INFO(get_logger(), "initializeRegistration end");
}

void PCLLocalization::initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "initialPoseReceived");
  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "initialpose_frame_id does not match global_frame_id");
    return;
  }
  initialpose_recieved_ = true;
  ndt_bootstrap_pending_ = true;
  have_good_pose_ = false;
  last_align_stamp_s_ = -1.0e9;
  corrent_pose_with_cov_stamped_ptr_ = msg;
  RCLCPP_INFO(get_logger(), "Initialpose accepted as NDT guess only; waiting for valid NDT before publishing map->odom");

  if (last_scan_ptr_) {
    cloudReceived(last_scan_ptr_);
  }
  RCLCPP_INFO(get_logger(), "initialPoseReceived end");
}

void PCLLocalization::mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  RCLCPP_INFO(get_logger(), "mapReceived");
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  if (msg->header.frame_id != global_frame_id_) {
    RCLCPP_WARN(this->get_logger(), "map_frame_id does not match　global_frame_id");
    return;
  }

  pcl::fromROSMsg(*msg, *map_cloud_ptr);

  if (registration_method_ == "GICP" || registration_method_ == "GICP_OMP") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    voxel_grid_filter_.setInputCloud(map_cloud_ptr);
    voxel_grid_filter_.filter(*filtered_cloud_ptr);
    registration_->setInputTarget(filtered_cloud_ptr);

  } else {
    registration_->setInputTarget(map_cloud_ptr);
  }

  map_recieved_ = true;
  RCLCPP_INFO(get_logger(), "mapReceived end");
}

void PCLLocalization::odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!use_odom_) {return;}
  if (!corrent_pose_with_cov_stamped_ptr_) {return;}

  double current_odom_received_time = stampSeconds(msg->header.stamp);
  if (last_odom_received_time_ < 0.0) {
    last_odom_received_time_ = current_odom_received_time;
    return;
  }
  double dt_odom = current_odom_received_time - last_odom_received_time_;
  last_odom_received_time_ = current_odom_received_time;
  if (dt_odom > 1.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is too large");
    return;
  }
  if (dt_odom < 0.0 /* [sec] */) {
    RCLCPP_WARN(this->get_logger(), "odom time interval is negative");
    return;
  }

  tf2::Quaternion previous_quat_tf;
  double roll, pitch, yaw;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation, previous_quat_tf);

  tf2::Matrix3x3(previous_quat_tf).getRPY(roll, pitch, yaw);

  roll += msg->twist.twist.angular.x * dt_odom;
  pitch += msg->twist.twist.angular.y * dt_odom;
  yaw += msg->twist.twist.angular.z * dt_odom;

  Eigen::Quaterniond quat_eig =
    Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

  geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(quat_eig);

  Eigen::Vector3d odom{
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z};
  Eigen::Vector3d delta_position = quat_eig.matrix() * dt_odom * odom;

  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x += delta_position.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y += delta_position.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z += delta_position.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
}

void PCLLocalization::imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
  if (!use_imu_) {return;}

  sensor_msgs::msg::Imu tf_converted_imu;

  try {
    const geometry_msgs::msg::TransformStamped transform = tfbuffer_.lookupTransform(
     base_frame_id_, msg->header.frame_id, tf2::TimePointZero);

    geometry_msgs::msg::Vector3Stamped angular_velocity, linear_acceleration, transformed_angular_velocity, transformed_linear_acceleration;
    geometry_msgs::msg::Quaternion  transformed_quaternion;

    angular_velocity.header = msg->header;
    angular_velocity.vector = msg->angular_velocity;
    linear_acceleration.header = msg->header;
    linear_acceleration.vector = msg->linear_acceleration;

    tf2::doTransform(angular_velocity, transformed_angular_velocity, transform);
    tf2::doTransform(linear_acceleration, transformed_linear_acceleration, transform);

    tf_converted_imu.angular_velocity = transformed_angular_velocity.vector;
    tf_converted_imu.linear_acceleration = transformed_linear_acceleration.vector;
    tf_converted_imu.orientation = transformed_quaternion;

  }
  catch (tf2::TransformException& ex)
  {
    std::cout << "Failed to lookup transform" << std::endl;
    RCLCPP_WARN(this->get_logger(), "Failed to lookup transform.");
    return;
  }

  Eigen::Vector3f angular_velo{tf_converted_imu.angular_velocity.x, tf_converted_imu.angular_velocity.y,
    tf_converted_imu.angular_velocity.z};
  Eigen::Vector3f acc{tf_converted_imu.linear_acceleration.x, tf_converted_imu.linear_acceleration.y, tf_converted_imu.linear_acceleration.z};
  Eigen::Quaternionf quat{msg->orientation.w, msg->orientation.x, msg->orientation.y,
    msg->orientation.z};
  double imu_time = msg->header.stamp.sec +
    msg->header.stamp.nanosec * 1e-9;

  lidar_undistortion_.getImu(angular_velo, acc, quat, imu_time);

}

void PCLLocalization::cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  last_scan_ptr_ = msg;
  if (!map_recieved_ || !initialpose_recieved_ || !corrent_pose_with_cov_stamped_ptr_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "NDT waiting: map=%d initialpose=%d pose=%d", map_recieved_, initialpose_recieved_,
      corrent_pose_with_cov_stamped_ptr_ != nullptr);
    return;
  }

  const double now_s = stampSeconds(msg->header.stamp);
  if (ndt_align_interval_s_ > 0.0 && now_s - last_align_stamp_s_ < ndt_align_interval_s_) {
    return;
  }
  // Record the attempt before align so a slow or failed NDT cannot be retried
  // on every incoming cloud and starve the odometry/TF callbacks.
  last_align_stamp_s_ = now_s;

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*msg, *cloud_ptr);
  const std::size_t raw_count = cloud_ptr->size();

  if (use_imu_) {
    lidar_undistortion_.adjustDistortion(cloud_ptr, now_s);
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
  voxel_grid_filter_.setInputCloud(cloud_ptr);
  voxel_grid_filter_.filter(*filtered_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZI>::Ptr source(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto & point : filtered_cloud_ptr->points) {
    const double range = std::hypot(point.x, point.y);
    if (range > scan_min_range_ && range < scan_max_range_) {
      source->push_back(point);
      for (int i = 1; i < std::max(1, far_point_boost_factor_); ++i) {
        if (far_point_boost_min_range_ > 0.0 && range >= far_point_boost_min_range_) {
          source->push_back(point);
        }
      }
    }
  }
  if (source->empty()) {
    RCLCPP_WARN(get_logger(), "NDT rejected before align: raw=%zu filtered=%zu source=0", raw_count, filtered_cloud_ptr->size());
    return;
  }
  registration_->setInputSource(source);

  Eigen::Affine3d affine;
  tf2::fromMsg(corrent_pose_with_cov_stamped_ptr_->pose.pose, affine);
  const Eigen::Matrix4f init_guess = affine.matrix().cast<float>();
  const Eigen::Quaterniond init_quat(init_guess.block<3, 3>(0, 0).cast<double>());

  pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  const auto time_align_start = std::chrono::steady_clock::now();
  registration_->align(*output_cloud, init_guess);
  const auto time_align_end = std::chrono::steady_clock::now();
  const double align_ms = std::chrono::duration<double, std::milli>(time_align_end - time_align_start).count();

  const bool has_converged = registration_->hasConverged();
  const double fitness_score = registration_->getFitnessScore();
  int iterations = -1;
  double probability = std::numeric_limits<double>::quiet_NaN();
  if (registration_method_ == "NDT_OMP") {
    auto ndt = boost::dynamic_pointer_cast<pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>>(registration_);
    if (ndt) {
      iterations = ndt->getFinalNumIteration();
      probability = ndt->getTransformationProbability();
    }
  }
  RCLCPP_INFO(get_logger(), "NDT align: stamp=%.3f raw=%zu filtered=%zu source=%zu time_ms=%.1f converged=%d fitness=%.6f iterations=%d probability=%.6f init=(%.3f,%.3f,%.3f)", now_s, raw_count, filtered_cloud_ptr->size(), source->size(), align_ms, has_converged, fitness_score, iterations, probability, init_guess(0, 3), init_guess(1, 3), init_guess(2, 3));

  if (!has_converged) {
    RCLCPP_WARN(get_logger(), "NDT rejected: not converged");
    return;
  }
  if (fitness_score > score_threshold_) {
    RCLCPP_WARN(get_logger(), "NDT rejected: fitness=%.6f threshold=%.6f", fitness_score, score_threshold_);
    return;
  }

  const Eigen::Matrix4f final_transformation = registration_->getFinalTransformation();
  Eigen::Quaterniond final_quat(final_transformation.block<3, 3>(0, 0).cast<double>());
  final_quat.normalize();
  const Eigen::Vector3d translation = final_transformation.block<3, 1>(0, 3).cast<double>();
  const Eigen::Vector3d init_translation = init_guess.block<3, 1>(0, 3).cast<double>();
  const double jump_translation = (translation - init_translation).norm();
  const double jump_rotation = quaternionAngle(init_quat, final_quat);
  if ((max_pose_jump_translation_ > 0.0 && jump_translation > max_pose_jump_translation_) ||
    (max_pose_jump_rotation_ > 0.0 && jump_rotation > max_pose_jump_rotation_)) {
    RCLCPP_WARN(get_logger(), "NDT map¡úodom jump rejected: translation=%.3f/%.3f rotation=%.3f/%.3f", jump_translation, max_pose_jump_translation_, jump_rotation, max_pose_jump_rotation_);
    return;
  }

  if (lock_planar_) {
    final_quat = Eigen::AngleAxisd(tf2::getYaw(corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation), Eigen::Vector3d::UnitZ());
  }
  const geometry_msgs::msg::Quaternion quat_msg = tf2::toMsg(final_quat);

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = now();
  tf_msg.header.frame_id = global_frame_id_;
  tf_msg.child_frame_id = base_frame_id_;
  tf_msg.transform.translation.x = translation.x();
  tf_msg.transform.translation.y = translation.y();
  tf_msg.transform.translation.z = translation.z();
  tf_msg.transform.rotation = quat_msg;

  // After this accepted result, the state becomes have_good_pose_ && !ndt_bootstrap_pending_.
  if (enable_map_odom_tf_) {
    try {
      const auto odom_to_base = tfbuffer_.lookupTransform(
        odom_frame_id_, base_frame_id_, tf2::TimePointZero);
      Eigen::Isometry3d map_to_base = Eigen::Isometry3d::Identity();
      map_to_base.linear() = final_transformation.block<3, 3>(0, 0).cast<double>();
      map_to_base.translation() = final_transformation.block<3, 1>(0, 3).cast<double>();
      const Eigen::Isometry3d odom_to_base_eigen = tf2::transformToEigen(odom_to_base);
      Eigen::Isometry3d map_to_odom = map_to_base * odom_to_base_eigen.inverse();
      if (have_map_odom_tf_) {
        const Eigen::Isometry3d previous = tf2::transformToEigen(last_map_odom_tf_);
        const double alpha = std::max(0.0, std::min(1.0, map_odom_smoothing_));
        const double correction_translation = (map_to_odom.translation() - previous.translation()).norm();
        const double correction_rotation = quaternionAngle(
          Eigen::Quaterniond(previous.rotation()), Eigen::Quaterniond(map_to_odom.rotation()));
        if ((max_map_odom_update_translation_ > 0.0 && correction_translation > max_map_odom_update_translation_) ||
          (max_map_odom_update_rotation_ > 0.0 && correction_rotation > max_map_odom_update_rotation_)) {
          RCLCPP_WARN(get_logger(), "NDT map¡úodom jump rejected: correction_translation=%.3f correction_rotation=%.3f", correction_translation, correction_rotation);
          return;
        }
        map_to_odom.translation() = (1.0 - alpha) * previous.translation() + alpha * map_to_odom.translation();
        Eigen::Quaterniond smoothed(previous.rotation());
        smoothed = smoothed.slerp(alpha, Eigen::Quaterniond(map_to_odom.rotation()));
        map_to_odom.linear() = smoothed.normalized().toRotationMatrix();
      }
      tf_msg.child_frame_id = odom_frame_id_;
      const Eigen::Quaterniond map_to_odom_quat(map_to_odom.rotation());
      tf_msg.transform.translation.x = map_to_odom.translation().x();
      tf_msg.transform.translation.y = map_to_odom.translation().y();
      tf_msg.transform.translation.z = map_to_odom.translation().z();
      tf_msg.transform.rotation = tf2::toMsg(map_to_odom_quat.normalized());
      last_map_odom_tf_ = tf_msg;
      have_map_odom_tf_ = true;
      RCLCPP_INFO(get_logger(), "NDT accepted: map->odom=(%.3f,%.3f,%.3f) jump=(%.3f m, %.3f rad)", tf_msg.transform.translation.x, tf_msg.transform.translation.y, tf_msg.transform.translation.z, jump_translation, jump_rotation);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "NDT rejected: waiting for %s->%s TF before publishing map->odom: %s", odom_frame_id_.c_str(), base_frame_id_.c_str(), ex.what());
      return;
    }
  }

  corrent_pose_with_cov_stamped_ptr_->header.stamp = msg->header.stamp;
  corrent_pose_with_cov_stamped_ptr_->header.frame_id = global_frame_id_;
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.x = translation.x();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.y = translation.y();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.position.z = translation.z();
  corrent_pose_with_cov_stamped_ptr_->pose.pose.orientation = quat_msg;
  have_good_pose_ = true;
  ndt_bootstrap_pending_ = false;
  pose_pub_->publish(*corrent_pose_with_cov_stamped_ptr_);

  broadcaster_.sendTransform(tf_msg);

  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = corrent_pose_with_cov_stamped_ptr_->header;
  pose_stamped.pose = corrent_pose_with_cov_stamped_ptr_->pose.pose;
  path_ptr_->poses.push_back(pose_stamped);
  path_pub_->publish(*path_ptr_);

  sensor_msgs::msg::PointCloud2 aligned_scan_msg;
  pcl::toROSMsg(*output_cloud, aligned_scan_msg);
  aligned_scan_msg.header.stamp = msg->header.stamp;
  aligned_scan_msg.header.frame_id = global_frame_id_;
  ndt_aligned_scan_pub_->publish(aligned_scan_msg);
  RCLCPP_INFO(get_logger(), "NDT aligned scan published: points=%zu", output_cloud->size());
}
