/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <angles/angles.h>
#include <tf2/utils.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <chrono>
#include <ndt_2d/ndt_model.hpp>
#include <ndt_2d/ndt_mapper.hpp>
#include <ndt_2d/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ndt_2d
{

Mapper::Mapper(const rclcpp::NodeOptions & options)
: rclcpp::Node("ndt_2d_mapper", options),
  map_update_available_(false),
  logger_(rclcpp::get_logger("ndt_2d_mapper"))
{
  map_resolution_ = this->declare_parameter<double>("resolution", 0.05);
  ndt_resolution_ = this->declare_parameter<double>("ndt_resolution", 0.25);
  minimum_travel_distance_ = this->declare_parameter<double>("minimum_travel_distance", 0.1);
  minimum_travel_rotation_ = this->declare_parameter<double>("minimum_travel_rotation", 1.0);
  rolling_depth_ = this->declare_parameter<int>("rolling_depth", 10);
  odom_frame_ = this->declare_parameter<std::string>("odom_frame", "odom");
  search_angular_resolution_ = this->declare_parameter<double>("search_angular_resolution", 0.0025);
  search_angular_size_ = this->declare_parameter<double>("search_angular_size", 0.1);
  search_linear_resolution_ = this->declare_parameter<double>("search_linear_resolution", 0.005);
  search_linear_size_ = this->declare_parameter<double>("search_linear_size", 0.05);

  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  tf2_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  grid_ = std::make_shared<OccupancyGrid>(map_resolution_);

  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", 1);
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "scan", 1, std::bind(&Mapper::laserCallback, this, std::placeholders::_1));

  map_publish_timer_ = this->create_wall_timer(std::chrono::milliseconds(250),
    std::bind(&Mapper::mapPublishCallback, this));
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
    RCLCPP_ERROR(logger_, "Could not transform %s to %s frame.",
                 msg->header.frame_id.c_str(), odom_frame_.c_str());
    return;
  }

  // Need to convert the scan into an ndt_2d style
  ScanPtr scan(new Scan());

  // Convert pose to ndt_2d style
  Pose2d odom_pose;
  odom_pose.x = odom_pose_tf.pose.position.x;
  odom_pose.y = odom_pose_tf.pose.position.y;
  odom_pose.theta = tf2::getYaw(odom_pose_tf.pose.orientation);

  // Make sure we have traveled far enough
  if (!odom_poses_.empty())
  {
    // Calculate delta in odometry frame
    double dx = odom_pose.x - odom_poses_.back().x;
    double dy = odom_pose.y - odom_poses_.back().y;
    double dth = angles::shortest_angular_distance(odom_poses_.back().theta, odom_pose.theta);
    double dist = (dx * dx) + (dy * dy);
    if (dist < minimum_travel_distance_ * minimum_travel_distance_ &&
        std::fabs(dth) < minimum_travel_rotation_)
    {
      return;
    }
    // Odometry and corrected frame may not be aligned - determine heading between them
    double heading = angles::shortest_angular_distance(odom_poses_.back().theta,
                                                       scans_.back()->pose.theta);
    // Now apply odometry delta, corrected by heading, to get initial corrected pose
    scan->pose.x = scans_.back()->pose.x + (dx * cos(heading)) - (dy * sin(heading));
    scan->pose.y = scans_.back()->pose.y + (dx * sin(heading)) + (dy * cos(heading));
    scan->pose.theta = angles::normalize_angle(scans_.back()->pose.theta + dth);
  }
  else
  {
    // Initialize corrected pose - start robot at origin of map
    scan->pose.x = 0.0;
    scan->pose.y = 0.0;
    scan->pose.theta = 0.0;
  }
  RCLCPP_INFO(logger_, "Adding scan to map");

  // Using this scan, convert ROS msg into ndt_2d style scan
  scan->points.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); ++i)
  {
    // Filter out NANs
    if (std::isnan(msg->ranges[i])) continue;
    // Project point and push into scan
    double angle = msg->angle_min + i * msg->angle_increment;
    Point point(cos(angle) * msg->ranges[i],
                sin(angle) * msg->ranges[i]);
    scan->points.push_back(point);
  }

  if (!odom_poses_.empty())
  {
    // Match new scan against last 10 scans
    auto end = scans_.end();
    auto begin = end;
    for (size_t i = 0; i < 10; ++i)
    {
      if (begin == scans_.begin()) break;
      --begin;
    }
    Pose2d correction;
    matchScans(begin, end, scan, correction);
    // Add correction to scan corrected pose
    scan->pose.x += correction.x;
    scan->pose.y += correction.y;
    scan->pose.theta += correction.theta;
  }

  std::cout << "Odom pose: " << odom_pose.x << " " << odom_pose.y
            << " " << odom_pose.theta << std::endl;
  std::cout << "Corrected: " << scan->pose.x << " " << scan->pose.y
            << " " << scan->pose.theta << std::endl;

  {
    // TODO(fergs): lock this when timer is converted to thread
    scans_.push_back(scan);
    odom_poses_.push_back(odom_pose);
    map_update_available_ = true;
  }
}

double Mapper::matchScans(const std::vector<ScanPtr>::iterator& scans_begin,
                          const std::vector<ScanPtr>::iterator& scans_end,
                          ScanPtr & scan, Pose2d & pose)
{
  // Build an NDT of the last several scans
  double ndt_size = 10.0;
  NDT ndt(ndt_resolution_, ndt_size, ndt_size, -0.5 * ndt_size, -0.5 * ndt_size);
  for (std::vector<ScanPtr>::const_iterator scan = scans_begin; scan != scans_end; ++scan)
  {
    ndt.addScan(*scan);
  }
  ndt.compute();

  // Search NDT for best correlation for new scan
  double best_score = 0;

  std::vector<Point> points_outer;
  std::vector<Point> points_inner;
  points_outer.resize(scan->points.size());
  points_inner.resize(scan->points.size());

  for (double dth = -search_angular_size_;
       dth < search_angular_size_;
       dth += search_angular_resolution_)
  {
    // Do orientation on the outer loop - then we can simply shift points in inner loops
    double costh = cos(scan->pose.theta + dth);
    double sinth = sin(scan->pose.theta + dth);
    for (size_t i = 0; i < points_outer.size(); ++i)
    {
      points_outer[i].x = scan->points[i].x * costh - scan->points[i].y * sinth + scan->pose.x;
      points_outer[i].y = scan->points[i].x * sinth + scan->points[i].y * costh + scan->pose.y;
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

        double likelihood = -ndt.likelihood(points_inner);
        if (likelihood < best_score)
        {
          best_score = likelihood;
          pose.x = dx;
          pose.y = dy;
          pose.theta = dth;
        }
      }
    }
  }

  return best_score;
}

void Mapper::publishTransform()
{
  // Latest corrected pose gives us map -> robot
  Pose2d & corrected = scans_.back()->pose;
  Eigen::Isometry3d map_to_robot(Eigen::Translation3d(corrected.x, corrected.y, 0.0) *
                                 Eigen::AngleAxisd(corrected.theta, Eigen::Vector3d::UnitZ()));

  // Latest odom pose gives us odom -> robot
  Pose2d & odom = odom_poses_.back();
  Eigen::Isometry3d odom_to_robot(Eigen::Translation3d(odom.x, odom.y, 0.0) *
                                  Eigen::AngleAxisd(odom.theta, Eigen::Vector3d::UnitZ()));

  // Compute map -> odom
  Eigen::Isometry3d map_to_odom(map_to_robot * odom_to_robot.inverse());

  // Publish TF between map and odom
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = this->now();
  transform.header.frame_id = "map";
  transform.child_frame_id = odom_frame_;
  transform.transform.translation.x = map_to_odom.translation().x();
  transform.transform.translation.y = map_to_odom.translation().y();
  Eigen::Quaterniond q(map_to_odom.rotation());
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();
  tf2_broadcaster_->sendTransform(transform);
}

// TODO(fergs): This should move to a separate thread
void Mapper::mapPublishCallback()
{
  if (!map_update_available_)
  {
    if (!odom_poses_.empty())
    {
      publishTransform();
    }
    // No map update to publish
    return;
  }

  {
    // TODO(fergs): lock this when timer is converted to thread
    map_update_available_ = false;
  }

  // Publish an occupancy grid
  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header.frame_id = "map";
  grid_msg.header.stamp = this->now();
  grid_->getMsg(scans_, grid_msg);
  map_pub_->publish(grid_msg);

  // Publish TF
  publishTransform();
}

}  // namespace ndt_2d

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ndt_2d::Mapper)
