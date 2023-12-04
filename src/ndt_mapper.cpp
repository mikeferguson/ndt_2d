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
  logger_(rclcpp::get_logger("ndt_2d_mapper")),
  prev_odom_pose_is_initialized_(true)
{
  map_resolution_ = this->declare_parameter<double>("resolution", 0.05);
  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 0.25);
  minimum_travel_distance_ = this->declare_parameter<double>("minimum_travel_distance", 0.1);
  minimum_travel_rotation_ = this->declare_parameter<double>("minimum_travel_rotation", 1.0);
  rolling_depth_ = this->declare_parameter<int>("rolling_depth", 10);
  robot_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  laser_max_beams_ = this->declare_parameter<int>("laser_max_beams", 100);
  search_angular_resolution_ = this->declare_parameter<double>("search_angular_resolution", 0.0025);
  search_angular_size_ = this->declare_parameter<double>("search_angular_size", 0.1);
  search_linear_resolution_ = this->declare_parameter<double>("search_linear_resolution", 0.005);
  search_linear_size_ = this->declare_parameter<double>("search_linear_size", 0.05);
  transform_timeout_ = this->declare_parameter<double>("transform_timeout", 0.2);
  global_search_size_ = this->declare_parameter<double>("global_search_size", 0.2);

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
    graph_ = std::make_shared<Graph>();
  }
  else
  {
    graph_ = std::make_shared<Graph>(map_file);
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
    graph_ = std::make_shared<Graph>(request->filename);
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
    ScanPtr scan(new Scan());
    scan->id = graph_->scans.size();
    scan->pose = fromMsg(msg->pose.pose);

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

    // range_max_ must be set before building the global_ndt_
    if (use_particle_filter_ || !enable_mapping_)
    {
      buildNDT(graph_->scans.begin(), graph_->scans.end(), global_ndt_);
    }
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

  // Need to convert the scan into an ndt_2d style
  ScanPtr scan(new Scan());
  scan->id = graph_->scans.size();

  // Convert pose to ndt_2d style
  Pose2d odom_pose = fromMsg(odom_pose_tf);

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
    scan->pose.x = prev_robot_pose_.x + (dx * cos(heading)) - (dy * sin(heading));
    scan->pose.y = prev_robot_pose_.y + (dx * sin(heading)) + (dy * cos(heading));
    scan->pose.theta = angles::normalize_angle(prev_robot_pose_.theta + dth);
  }
  else
  {
    // Initialize corrected pose - start robot at origin of map
    scan->pose.x = 0.0;
    scan->pose.y = 0.0;
    scan->pose.theta = 0.0;
  }

  // Minor optimization
  double cos_lt = cos(laser_transform_.theta);
  double sin_lt = sin(laser_transform_.theta);

  // Using this scan, convert ROS msg into ndt_2d style scan
  scan->points.reserve(msg->ranges.size());
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
    filter_->measure(global_ndt_, scan);
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

      // Build NDT of rolling window scans
      std::shared_ptr<NDT> ndt;
      buildNDT(rolling, graph_->scans.end(), ndt);

      // Local consistency - match new scan against last 10 scans
      Pose2d correction;
      Eigen::Matrix3d covariance;
      double uncorrected_score = ndt->likelihood(scan);
      matchScan(ndt, scan, correction, covariance, laser_max_beams_);
      matched_score = ndt->likelihood(scan, correction);
      RCLCPP_INFO(logger_, "           %f, %f, %f (%f | %f)",
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
    double uncorrected_score = global_ndt_->likelihood(scan);
    matchScan(global_ndt_, scan, correction, covariance, laser_max_beams_);
    double score = global_ndt_->likelihood(scan, correction);
    RCLCPP_INFO(logger_, "           %f, %f, %f (%f | %f)",
                correction.x, correction.y, correction.theta, uncorrected_score, score);

    // Add correction to scan corrected pose
    scan->pose.x += correction.x;
    scan->pose.y += correction.y;
    scan->pose.theta += correction.theta;

    prev_odom_pose_ = odom_pose;
    prev_robot_pose_ = scan->pose;
  }
}

void Mapper::buildNDT(const std::vector<ScanPtr>::const_iterator& begin,
                      const std::vector<ScanPtr>::const_iterator& end,
                      std::shared_ptr<NDT> & ndt)
{
  // Compute bounding box required
  double min_x_ = std::numeric_limits<double>::max();
  double max_x_ = std::numeric_limits<double>::min();
  double min_y_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::min();
  for (auto scan = begin; scan != end; ++scan)
  {
    min_x_ = std::min((*scan)->pose.x - range_max_, min_x_);
    max_x_ = std::max((*scan)->pose.x + range_max_, max_x_);
    min_y_ = std::min((*scan)->pose.y - range_max_, min_y_);
    max_y_ = std::max((*scan)->pose.y + range_max_, max_y_);
  }

  ndt = std::make_shared<NDT>(ndt_resolution_,
                              (max_x_ - min_x_), (max_y_ - min_y_), min_x_, min_y_);
  for (auto scan = begin; scan != end; ++scan)
  {
    ndt->addScan(*scan);
  }

  ndt->compute();
}

double Mapper::matchScan(const std::shared_ptr<NDT> & ndt,
                         const ScanPtr & scan, Pose2d & pose,
                         Eigen::Matrix3d & covariance,
                         size_t scan_points_to_use)
{
  // Search NDT for best correlation for new scan
  double best_score = 0;

  // Working values for covariance computation
  Eigen::Matrix3d k = Eigen::Matrix3d::Zero();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  double s = 0.0;

  // Subsample the scan
  scan_points_to_use = std::min(scan_points_to_use, scan->points.size());
  double scan_step = static_cast<double>(scan->points.size()) / scan_points_to_use;

  std::vector<Point> points_outer;
  std::vector<Point> points_inner;
  points_outer.resize(scan_points_to_use);
  points_inner.resize(scan_points_to_use);

  for (double dth = -search_angular_size_;
       dth < search_angular_size_;
       dth += search_angular_resolution_)
  {
    // Do orientation on the outer loop - then we can simply shift points in inner loops
    double costh = cos(scan->pose.theta + dth);
    double sinth = sin(scan->pose.theta + dth);
    for (size_t i = 0; i < points_outer.size(); ++i)
    {
      size_t scan_idx = static_cast<size_t>(i * scan_step);
      points_outer[i].x = scan->points[scan_idx].x * costh -
                          scan->points[scan_idx].y * sinth + scan->pose.x;
      points_outer[i].y = scan->points[scan_idx].x * sinth +
                          scan->points[scan_idx].y * costh + scan->pose.y;
    }

    for (double dx = -search_linear_size_;
         dx < search_linear_size_;
         dx += search_linear_resolution_)
    {
      for (double dy = -search_linear_size_;
           dy < search_linear_size_;
           dy += search_linear_resolution_)
      {
        for (size_t i = 0; i < points_inner.size(); ++i)
        {
          points_inner[i].x = points_outer[i].x + dx;
          points_inner[i].y = points_outer[i].y + dy;
        }

        double likelihood = -ndt->likelihood(points_inner);
        if (likelihood < best_score)
        {
          best_score = likelihood;
          pose.x = dx;
          pose.y = dy;
          pose.theta = dth;
        }

        // Covariance computation
        Eigen::Vector3d x(dx, dy, dth);
        k += x * x.transpose() * likelihood;
        u += x * likelihood;
        s += likelihood;
      }
    }
  }

  // Compute covariance
  covariance = (1 / s) * k + (1 / (s * s) * u * u.transpose());

  return best_score;
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
    std::shared_ptr<NDT> ndt;
    buildNDT(begin, end, ndt);

    // Try to match scans
    Pose2d correction;
    Eigen::Matrix3d covariance;
    matchScan(ndt, scan, correction, covariance, laser_max_beams_);
    double score = ndt->likelihood(scan, correction);

    if (score > uncorrected_score)
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

      // TODO(fergs): only run this occasionally?
      solver_->optimize(graph_->constraints, graph_->scans);
    }
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
