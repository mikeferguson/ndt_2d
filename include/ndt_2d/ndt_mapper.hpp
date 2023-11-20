/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__NDT_MAPPER_HPP_
#define NDT_2D__NDT_MAPPER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

namespace ndt_2d
{

class Mapper : public rclcpp::Node
{
public:
  explicit Mapper(const rclcpp::NodeOptions & options);
  virtual ~Mapper();

protected:
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

  // Parameters
  double map_resolution_;
  double ndt_resolution_;
  double minimum_travel_distance_, minimum_travel_rotation_;
  std::string odom_frame_;

  // ROS 2 interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Map data
  std::vector<ScanPtr> scans_;
  std::vector<Pose2d> odom_poses_;
  std::vector<Pose2d> corrected_poses_;
  Pose2d last_correction_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__NDT_MAPPER_HPP_
