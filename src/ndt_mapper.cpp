/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <angles/angles.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>
#include <iostream>
#include <ndt_2d/conversions.hpp>
#include <ndt_2d/ndt_model.hpp>
#include <ndt_2d/ndt_mapper.hpp>
#include <ndt_2d/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ndt_2d
{

Mapper::Mapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("ndt_2d_mapper", options),
  map_update_available_(false),
  laser_inverted_(false),
  optimization_last_(0),
  logger_(rclcpp::get_logger("ndt_2d_mapper")),
  prev_odom_pose_is_initialized_(true)
{
  map_resolution_ = this->declare_parameter<double>("resolution", 0.05);
  minimum_travel_distance_ = this->declare_parameter<double>("minimum_travel_distance", 0.1);
  minimum_travel_rotation_ = this->declare_parameter<double>("minimum_travel_rotation", 1.0);
  rolling_depth_ = this->declare_parameter<int>("rolling_depth", 10);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  laser_max_beams_ = this->declare_parameter<int>("laser_max_beams", 100);
  transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.2);
  use_barycenter_ = this->declare_parameter<bool>("use_barycenter", true);
  global_search_size_ = this->declare_parameter<double>("global_search_size", 0.2);
  global_search_limit_ = this->declare_parameter<int>("global_search_limit", 3);
  optimization_node_limit_ = this->declare_parameter<int>("optimization_node_limit", 25);

  use_particle_filter_ = this->declare_parameter<bool>("use_particle_filter", false);
  if (use_particle_filter_)
  {
    double a1 = this->declare_parameter<double>("odom_alpha1", 0.2);
    double a2 = this->declare_parameter<double>("odom_alpha2", 0.2);
    double a3 = this->declare_parameter<double>("odom_alpha3", 0.2);
    double a4 = this->declare_parameter<double>("odom_alpha4", 0.2);
    double a5 = this->declare_parameter<double>("odom_alpha5", 0.2);
    MotionModelPtr model = std::make_shared<MotionModel>(a1, a2, a3, a4, a5);

    size_t min_p = this->declare_parameter<int>("min_particles", 100);
    size_t max_p = this->declare_parameter<int>("max_particles", 500);

    kld_err_ = this->declare_parameter<double>("kld_err", 0.01);
    kld_z_ = this->declare_parameter<double>("kld_z", 0.99);

    filter_ = std::make_shared<ParticleFilter>(min_p, max_p, model);
  }

  enable_mapping_ = this->declare_parameter<bool>("enable_mapping", true);

  solver_ = std::make_shared<CeresSolver>();

  // Negative value indicates that we should use the sensor max range
  range_max_ = this->declare_parameter<double>("max_range", -1.0);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  double occ_thresh = this->declare_parameter<double>("occupancy_threshold", 0.25);
  grid_ = std::make_shared<OccupancyGrid>(map_resolution_, occ_thresh);

  std::string map_file = this->declare_parameter<std::string>("map_file", "");
  if (map_file.empty())
  {
    graph_ = std::make_shared<Graph>(use_barycenter_);
  }
  else
  {
    graph_ = std::make_shared<Graph>(use_barycenter_, map_file);
    prev_odom_pose_is_initialized_ = false;
    map_update_available_ = true;
  }

  configure_srv_ = this->create_service<ndt_2d::srv::Configure>("configure",
    std::bind(&Mapper::configure, this, std::placeholders::_1, std::placeholders::_2));

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  graph_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "graph", rclcpp::SystemDefaultsQoS());

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::SystemDefaultsQoS(),
    std::bind(&Mapper::poseCallback, this, std::placeholders::_1));

  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", rclcpp::SystemDefaultsQoS(),
    std::bind(&Mapper::laserCallback, this, std::placeholders::_1));

  if (use_particle_filter_)
  {
    particle_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
      "particlecloud", rclcpp::SystemDefaultsQoS());
  }

  map_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(250),
    std::bind(&Mapper::mapPublishCallback, this));
}

Mapper::~Mapper()
{
}

void Mapper::configure(const std::shared_ptr<srv::Configure::Request> request,
                       std::shared_ptr<srv::Configure::Response>)
{
  // Enable/disable
  if (request->action & srv::Configure::Request::ENABLE_MAPPING)
  {
    RCLCPP_INFO(logger_, "Enabling mapping");
    enable_mapping_ = true;
  }
  else if (request->action & srv::Configure::Request::DISABLE_MAPPING)
  {
    RCLCPP_INFO(logger_, "Disabling mapping");
    enable_mapping_ = false;
    // Need to be relocalized before mapping can continue
    prev_odom_pose_is_initialized_ = false;
  }

  if (request->action & srv::Configure::Request::LOAD_FROM_FILE)
  {
    RCLCPP_INFO(logger_, "Loading map from %s", request->filename.c_str());
    graph_ = std::make_shared<Graph>(use_barycenter_, request->filename);
    map_update_available_ = true;
    prev_odom_pose_is_initialized_ = false;
  }
  else if (request->action & srv::Configure::Request::SAVE_TO_FILE)
  {
    RCLCPP_INFO(logger_, "Saving map to %s", request->filename.c_str());
    graph_->save(request->filename);
  }
}

void Mapper::poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg)
{
  if (enable_mapping_ && prev_odom_pose_is_initialized_)
  {
    RCLCPP_WARN(logger_, "Ignoring initial pose, already mapping");
    return;
  }

  if (msg->header.frame_id != "map")
  {
    RCLCPP_ERROR(logger_, "Cannot set initial pose in frames other than map");
    return;
  }

  // Find transform from odom to robot frames
  geometry_msgs::msg::TransformStamped odom_to_robot;
  try
  {
    odom_to_robot = tf2_buffer_->lookupTransform(odom_frame_, robot_frame_, tf2::TimePointZero);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                 odom_frame_.c_str(), robot_frame_.c_str());
    return;
  }

  if (use_particle_filter_)
  {
    // Initialize particle filter
    filter_->init(msg->pose.pose.position.x,
                  msg->pose.pose.position.y,
                  tf2::getYaw(msg->pose.pose.orientation),
                  sqrt(msg->pose.covariance[0]),
                  sqrt(msg->pose.covariance[7]),
                  sqrt(msg->pose.covariance[35]));
    // Publish visualization
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    filter_->getMsg(msg);
    particle_pub_->publish(msg);
  }
  else if (enable_mapping_)
  {
    // If mapping, connect this pose to the graph
    ScanPtr scan = std::make_shared<Scan>();
    scan->id = graph_->scans.size();
    scan->pose = fromMsg(msg->pose.pose);
    scan->update();

    // Find the closest node in existing graph, then add scan to graph
    std::vector<size_t> nearest = graph_->findNearest(scan);
    if (nearest.empty())
    {
      RCLCPP_ERROR(logger_, "Cannot localize robot, not close enough to existing graph");
      return;
    }
    graph_->scans.push_back(scan);

    // Add a constraint
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    covariance(0, 0) = msg->pose.covariance[0];
    covariance(1, 1) = msg->pose.covariance[7];
    covariance(2, 2) = msg->pose.covariance[35];
    ConstraintPtr constraint = makeConstraint(graph_->scans[nearest[0]], scan, covariance);
    graph_->constraints.push_back(constraint);
  }

  // Localize robot
  prev_robot_pose_ = fromMsg(msg->pose.pose);
  prev_odom_pose_ = fromMsg(odom_to_robot);
  prev_odom_pose_is_initialized_ = true;

  RCLCPP_INFO(logger_, "Localized to %f, %f, %f", prev_robot_pose_.x,
              prev_robot_pose_.y, prev_robot_pose_.theta);
}

void Mapper::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  // Save data from laser scan message
  if (range_max_ < 0) range_max_ = msg->range_max;
  if (laser_frame_ == "")
  {
    try
    {
      // Determine pose of laser relative to base
      auto t = tf2_buffer_->lookupTransform(robot_frame_, msg->header.frame_id, tf2::TimePointZero);
      laser_transform_ = fromMsg(t);
      if (std::fabs(t.transform.rotation.x) > 0.02 ||
          std::fabs(t.transform.rotation.y) > 0.02)
      {
        RCLCPP_WARN(logger_, "Treating laser as inverted");
        laser_inverted_ = true;
      }
      RCLCPP_INFO(logger_, "Robot -> laser transform: %f, %f, %f",
                           laser_transform_.x, laser_transform_.y, laser_transform_.theta);
    }
    catch (const tf2::TransformException& ex)
    {
      RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                   robot_frame_.c_str(), msg->header.frame_id.c_str());
      return;
    }
    laser_frame_ = msg->header.frame_id;

    // range_max_ must be set before building the global scan matcher NDT
    if (use_particle_filter_ || !enable_mapping_)
    {
      global_scan_matcher_ = std::make_shared<ScanMatcherNDT>(this);
      global_scan_matcher_->setRangeMax(range_max_);
      global_scan_matcher_->addScans(graph_->scans.begin(), graph_->scans.end());
    }

    local_scan_matcher_ = std::make_shared<ScanMatcherNDT>(this);
    local_scan_matcher_->setRangeMax(range_max_);
  }

  // If we have loaded a previous map, need to localize first
  if (!prev_odom_pose_is_initialized_)
  {
    RCLCPP_WARN(logger_, "Can not handle scan, not localized within map");
    return;
  }

  // Find pose of the robot in odometry frame
  geometry_msgs::msg::PoseStamped odom_pose_tf;
  odom_pose_tf.header.stamp = msg->header.stamp;
  odom_pose_tf.header.frame_id = robot_frame_;
  odom_pose_tf.pose.orientation.w = 1.0;
  try
  {
    tf2_buffer_->transform(odom_pose_tf, odom_pose_tf, odom_frame_,
                           tf2::durationFromSec(transform_timeout_));
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                 robot_frame_.c_str(), odom_frame_.c_str());
    return;
  }

  // Convert pose to ndt_2d style
  Pose2d odom_pose = fromMsg(odom_pose_tf);
  Pose2d robot_pose;

  // Make sure we have traveled far enough
  if (!graph_->scans.empty())
  {
    // Calculate delta in odometry frame
    double dx = odom_pose.x - prev_odom_pose_.x;
    double dy = odom_pose.y - prev_odom_pose_.y;
    double dth = angles::shortest_angular_distance(prev_odom_pose_.theta, odom_pose.theta);
    double dist = (dx * dx) + (dy * dy);
    if (dist < minimum_travel_distance_ * minimum_travel_distance_ &&
        std::fabs(dth) < minimum_travel_rotation_)
    {
      return;
    }

    // Odometry frame is usually not aligned with map frame
    double heading = angles::shortest_angular_distance(prev_odom_pose_.theta,
                                                       prev_robot_pose_.theta);
    // Now apply odometry delta, corrected by heading, to get initial corrected pose
    robot_pose.x = prev_robot_pose_.x + (dx * cos(heading)) - (dy * sin(heading));
    robot_pose.y = prev_robot_pose_.y + (dx * sin(heading)) + (dy * cos(heading));
    robot_pose.theta = angles::normalize_angle(prev_robot_pose_.theta + dth);
  }

  // Need to convert the scan into an ndt_2d style
  ScanPtr scan = std::make_shared<Scan>();
  scan->id = graph_->scans.size();
  scan->pose = robot_pose;
  scan->points.reserve(msg->ranges.size());

  // Minor optimization
  double cos_lt = cos(laser_transform_.theta);
  double sin_lt = sin(laser_transform_.theta);

  // Using this scan, convert ROS msg into ndt_2d style scan
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    // Filter out NANs and scans beyond max range
    if (std::isnan(msg->ranges[i]) || msg->ranges[i] > range_max_) continue;
    // Project point in laser frame
    double angle = (msg->angle_min + i * msg->angle_increment);
    if (laser_inverted_) angle *= -1.0;
    Point lp(cos(angle) * msg->ranges[i],
             sin(angle) * msg->ranges[i]);
    // Transform to robot frame
    Point point(cos_lt * lp.x - sin_lt * lp.y + laser_transform_.x,
                sin_lt * lp.x + cos_lt * lp.y + laser_transform_.y);
    // Add point to scan
    scan->points.push_back(point);
  }
  scan->update();

  if (use_particle_filter_)
  {
    // Extract change in position in map frame
    Eigen::Vector3d map_delta(scan->pose.x - prev_robot_pose_.x,
                              scan->pose.y - prev_robot_pose_.y,
                              1.0);
    // Transform change in pose into robot-centric frame
    Eigen::Isometry3d transform(Eigen::Translation3d(0.0, 0.0, 0.0) *
                                Eigen::AngleAxisd(prev_robot_pose_.theta,
                                                  Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d robot_delta = transform.inverse() * map_delta;
    // Compute change in heading
    robot_delta(2) = angles::shortest_angular_distance(prev_robot_pose_.theta,
                                                       scan->pose.theta);

    RCLCPP_INFO(logger_, "Updating filter with control %f %f %f", robot_delta(0),
                         robot_delta(1), robot_delta(2));

    filter_->update(robot_delta(0), robot_delta(1), robot_delta(2));
    filter_->measure(global_scan_matcher_, scan);
    filter_->resample(kld_err_, kld_z_);

    auto mean = filter_->getMean();
    scan->pose.x = mean(0);
    scan->pose.y = mean(1);
    scan->pose.theta = mean(2);

    // Publish visualization
    geometry_msgs::msg::PoseArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = this->now();
    filter_->getMsg(msg);
    particle_pub_->publish(msg);

    RCLCPP_INFO(logger_, "New pose: %f, %f, %f", scan->pose.x, scan->pose.y, scan->pose.theta);

    prev_odom_pose_ = odom_pose;
    prev_robot_pose_ = scan->pose;
  }
  else if (enable_mapping_)
  {
    RCLCPP_INFO(logger_, "Adding scan to map");
    RCLCPP_INFO(logger_, "Odom pose: %f, %f, %f", odom_pose.x, odom_pose.y, odom_pose.theta);

    double matched_score = 0.0;
    if (!graph_->scans.empty())
    {
      // Determine rolling window
      size_t start = (graph_->scans.size() <= rolling_depth_) ? 0 : graph_->scans.size() - 10;
      auto rolling = graph_->scans.begin() + start;

      // Create scan matcher with rolling window scans
      local_scan_matcher_->reset();
      local_scan_matcher_->addScans(rolling, graph_->scans.end());

      // Local consistency - match new scan against last 10 scans
      Pose2d correction;
      Eigen::Matrix3d covariance;
      double uncorrected_score = local_scan_matcher_->scoreScan(scan);
      local_scan_matcher_->matchScan(scan, correction, covariance, laser_max_beams_);
      matched_score = local_scan_matcher_->scoreScan(scan, correction);
      RCLCPP_INFO(logger_, "           %f, %f, %f (%f -> %f)",
                  correction.x, correction.y, correction.theta, uncorrected_score, matched_score);

      // Add correction to scan corrected pose
      scan->pose.x += correction.x;
      scan->pose.y += correction.y;
      scan->pose.theta += correction.theta;

      // Add odom constraint to the graph
      ConstraintPtr constraint = makeConstraint(graph_->scans.back(), scan, covariance);
      graph_->constraints.push_back(constraint);
    }

    RCLCPP_INFO(logger_, "Corrected: %f, %f, %f", scan->pose.x, scan->pose.y, scan->pose.theta);

    // Append new scan to our graph
    graph_->scans.push_back(scan);
    prev_odom_pose_ = odom_pose;
    prev_robot_pose_ = scan->pose;
    map_update_available_ = true;

    // Global consistency - search scans for global matches
    searchGlobalMatches(scan, matched_score);
  }
  else
  {
    // Not using particle filter, nor mapping, just track motion of robot
    Pose2d correction;
    Eigen::Matrix3d covariance;
    double uncorrected_score = global_scan_matcher_->scoreScan(scan);
    global_scan_matcher_->matchScan(scan, correction, covariance, laser_max_beams_);
    double score = global_scan_matcher_->scoreScan(scan, correction);
    RCLCPP_INFO(logger_, "           %f, %f, %f (%f -> %f)",
                correction.x, correction.y, correction.theta, uncorrected_score, score);

    // Add correction to scan corrected pose
    scan->pose.x += correction.x;
    scan->pose.y += correction.y;
    scan->pose.theta += correction.theta;

    prev_odom_pose_ = odom_pose;
    prev_robot_pose_ = scan->pose;
  }
}

void Mapper::searchGlobalMatches(ScanPtr & scan, double uncorrected_score)
{
  // Can't do a global search until we have at least enough scans
  if (graph_->scans.size() <= rolling_depth_)
  {
    return;
  }

  // Determine where rolling window starts
  size_t rolling = graph_->scans.size() - rolling_depth_;

  std::vector<size_t> scans = graph_->findNearest(scan, global_search_size_, rolling);
  size_t num_scans_to_check = global_search_limit_;
  for (auto i : scans)
  {
    const auto & candidate = graph_->scans[i];
    if (candidate->points.empty()) continue;

    // Take one additional scan on either side of candidate
    size_t begin_idx = (i > 0) ? i - 1: i;
    size_t end_idx = (i < rolling) ? i + 1 : i;
    auto begin = graph_->scans.begin() + begin_idx;
    auto end = graph_->scans.begin() + end_idx;

    // Build NDT of candidate region
    local_scan_matcher_->reset();
    local_scan_matcher_->addScans(begin, end);

    // Try to match scans
    Pose2d correction;
    Eigen::Matrix3d covariance;
    local_scan_matcher_->matchScan(scan, correction, covariance, laser_max_beams_);
    double score = local_scan_matcher_->scoreScan(scan, correction);

    if (score < uncorrected_score)
    {
      RCLCPP_INFO(logger_, "Adding loop closure from %lu to %lu (score %f -> %f)",
                           candidate->id, scan->id, uncorrected_score, score);

      // Correct pose
      scan->pose.x += correction.x;
      scan->pose.y += correction.y;
      scan->pose.theta += correction.theta;

      // Add constraint to the graph
      ConstraintPtr constraint = makeConstraint(candidate, scan, covariance);
      constraint->switchable = true;
      graph_->constraints.push_back(constraint);

      if (graph_->scans.size() - optimization_last_ > optimization_node_limit_)
      {
        RCLCPP_INFO(logger_, "Optimizing pose graph");
        solver_->optimize(graph_->constraints, graph_->scans);
        optimization_last_ = graph_->scans.size();
      }
    }

    if (--num_scans_to_check == 0) break;
  }
}

void Mapper::publishTransform()
{
  // prev_robot_pose_ pose gives us map -> robot
  Eigen::Isometry3d map_to_robot = toEigen(prev_robot_pose_);

  // Latest odom pose gives us odom -> robot
  Eigen::Isometry3d odom_to_robot = toEigen(prev_odom_pose_);

  // Compute map -> odom
  Eigen::Isometry3d map_to_odom(map_to_robot * odom_to_robot.inverse());

  // Publish TF between map and odom
  geometry_msgs::msg::TransformStamped transform = tf2::eigenToTransform(map_to_odom);
  transform.header.stamp = this->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = odom_frame_;
  tf2_broadcaster_->sendTransform(transform);
}

// TODO(fergs): This should move to a separate thread, with proper locking
void Mapper::mapPublishCallback()
{
  if (!map_update_available_)
  {
    if (!graph_->scans.empty())
    {
      publishTransform();
    }
    // No map update to publish
    return;
  }

  map_update_available_ = false;
  rclcpp::Time now = this->now();

  // Publish an occupancy grid
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = "map";
  grid_msg.header.stamp = now;
  grid_msg.info.map_load_time = now;
  grid_->getMsg(graph_->scans, grid_msg);
  map_pub_->publish(grid_msg);

  // Publish the graph
  visualization_msgs::msg::MarkerArray graph_msg;
  graph_->getMsg(graph_msg, now);
  graph_pub_->publish(graph_msg);

  // Publish TF
  publishTransform();
}

}  // namespace ndt_2d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_2d::Mapper)
