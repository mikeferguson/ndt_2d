/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <angles/angles.h>
#include <tf2/utils.h>
#include <cmath>
#include <ndt_2d/ndt_model.hpp>
#include <ndt_2d/ndt_mapper.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ndt_2d
{

Mapper::Mapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("ndt_2d_mapper", options),
  logger_(rclcpp::get_logger("ndt_2d_mapper"))
{
  map_resolution_ = this->declare_parameter<double>("resolution", 0.05);
  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 0.25);
  minimum_travel_distance_ = this->declare_parameter<double>("minimum_travel_distance", 0.1);
  minimum_travel_rotation_ = this->declare_parameter<double>("minimum_travel_rotation", 1.0);
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Mapper::laserCallback, this, std::placeholders::_1));
}

Mapper::~Mapper()
{
}

void Mapper::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  // Find pose of the robot in odometry frame
  geometry_msgs::msg::PoseStamped odom_pose_tf;
  odom_pose_tf.header = msg->header;
  odom_pose_tf.pose.orientation.w = 1.0;
  try
  {
    tf2_buffer_->transform(odom_pose_tf, odom_pose_tf, odom_frame_);
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_ERROR(logger_, "Could not transform odom pose.");
    return;
  }

  // Convert pose to ndt_2d style
  Pose2d odom_pose, corrected_pose;
  odom_pose.x = odom_pose_tf.pose.position.x;
  odom_pose.y = odom_pose_tf.pose.position.y;
  odom_pose.theta = tf2::getYaw(odom_pose_tf.pose.orientation);

  // Make sure we have traveled far enough
  if (!odom_poses_.empty())
  {
    double dx = odom_pose.x - odom_poses_.back().x;
    double dy = odom_pose.y - odom_poses_.back().y;
    double dist = (dx * dx) + (dy * dy);
    double rot = angles::shortest_angular_distance(odom_poses_.back().theta, odom_pose.theta);
    if (dist < minimum_travel_distance_ * minimum_travel_distance_ &&
        std::fabs(rot) < minimum_travel_rotation_)
    {
      return;
    }
    // Going to use this pose, apply last correction
    corrected_pose.x = odom_pose.x - last_correction_.x;
    corrected_pose.y = odom_pose.y - last_correction_.y;
    corrected_pose.theta = odom_pose.theta - last_correction_.theta;
  }
  else
  {
    // Initialize corrections
    corrected_pose.x = 0.0;
    corrected_pose.y = 0.0;
    corrected_pose.theta = 0.0;
    last_correction_.x = -odom_pose.x;
    last_correction_.y = -odom_pose.y;
    last_correction_.theta = -odom_pose.theta;
  }
  RCLCPP_INFO(logger_, "Adding scan to map");

  // Convert ROS msg into NDT scan
  ScanPtr scan(new Scan());
  scan->points.reserve(msg->ranges.size());

  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    // Filter out NANs
    if (std::isnan(msg->ranges[i])) continue;
    // Project point and push into scan
    double angle = msg->angle_min + i * msg->angle_increment;
    Point point(sin(angle) * msg->ranges[i],
                cos(angle) * msg->ranges[i]);
    scan->points.push_back(point);
  }

  // Add scan, odom_pose
  scans_.push_back(scan);
  odom_poses_.push_back(odom_pose);

  // Build the NDT
  NDT ndt(ndt_resolution_, 10.0, 10.0, -5.0, -5.0);
  ndt.addScan(scan, odom_pose);
  ndt.compute();

  // Add the corrected pose
  corrected_poses_.push_back(corrected_pose);
  last_correction_ = corrected_pose;

  // TODO(fergs): This should move to a separate thread and only publish periodically
  // Build map message by sampling from NDT
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = odom_frame_;
  grid.header.stamp = this->now();
  grid.info.resolution = map_resolution_;
  grid.info.width = 10.0 / map_resolution_;
  grid.info.height = 10.0 / map_resolution_;
  grid.info.origin.position.x = -5.0;
  grid.info.origin.position.y = -5.0;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (size_t x = 0; x < grid.info.width; ++x)
  {
    for (size_t y = 0; y < grid.info.height; ++y)
    {
      double mx = x * map_resolution_ - 5.0;
      double my = y * map_resolution_ - 5.0;
      // This needs to be updated once NDT handles origin properly
      ndt_2d::Point point(mx, my);
      double l = ndt.likelihood(point);
      if (l > 0.0 && l < 50.0)
      {
        grid.data[y + (x * grid.info.height)] = 100;
      }
    }
  }
  map_pub_->publish(grid);
}

}  // namespace ndt_2d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_2d::Mapper)
