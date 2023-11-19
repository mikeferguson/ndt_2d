/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <cmath>
#include <ndt_2d/ndt_model.hpp>
#include <ndt_2d/ndt_mapper.hpp>

namespace ndt_2d
{

Mapper::Mapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("ndt_2d_mapper", options),
  logger_(rclcpp::get_logger("ndt_2d_mapper"))
{
  resolution_ = this->declare_parameter<double>("resolution", 0.05);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Mapper::laserCallback, this, std::placeholders::_1));
}

Mapper::~Mapper()
{
}

void Mapper::laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg)
{
  // Convert ROS msg into NDT scan
  ndt_2d::ScanPtr scan(new ndt_2d::Scan());
  scan->points.reserve(msg->ranges.size());

  double angle = msg->angle_min;
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    ndt_2d::Point point(sin(angle) * msg->ranges[i],
                        cos(angle) * msg->ranges[i]);
    scan->points.push_back(point);
    angle += msg->angle_increment;
  }

  // Build the NDT
  ndt_2d::NDT ndt(0.25, 25.0, 25.0);
  ndt_2d::Pose2d pose;
  pose.x = 12.5;
  pose.y = 12.5;
  ndt.addScan(scan, pose);
  ndt.compute();

  // Build map message by sampling from NDT
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "base_link";
  grid.header.stamp = this->now();
  grid.info.resolution = resolution_;
  grid.info.width = 25.0 / resolution_;
  grid.info.height = 25.0 / resolution_;
  grid.info.origin.position.x = -12.5;
  grid.info.origin.position.y = -12.5;
  grid.data.assign(grid.info.width * grid.info.height, 0);
  for (size_t x = 0; x < grid.info.width; ++x)
  {
    for (size_t y = 0; y < grid.info.height; ++y)
    {
      // This needs to be updated once NDT handles origin properly
      ndt_2d::Point point(x + 12.5, y + 12.5);
      if (ndt.likelihood(point) > 0.5)
      {
        grid.data[x + (y * grid.info.width)] = 100;
      }
    }
  }
  map_pub_->publish(grid);
}

}  // namespace ndt_2d
