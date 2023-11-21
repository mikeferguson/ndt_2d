/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__NDT_MAPPER_HPP_
#define NDT_2D__NDT_MAPPER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ndt_2d/occupancy_grid.hpp>
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
  void publishTransform();
  void mapPublishCallback();
  bool map_update_available_;

  // Parameters
  double map_resolution_;
  double ndt_resolution_;
  double minimum_travel_distance_, minimum_travel_rotation_;
  size_t rolling_depth_;
  std::string odom_frame_;

  // ROS 2 interfaces
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  // Map data
  // Vector of scans used to build the map
  std::vector<ScanPtr> scans_;
  // The odometry pose of each scan
  std::vector<Pose2d> odom_poses_;
  // The corrected pose of each scan based on scan matcher, loop closure
  std::vector<Pose2d> corrected_poses_;

  // Map export
  OccupancyGridPtr grid_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__NDT_MAPPER_HPP_
