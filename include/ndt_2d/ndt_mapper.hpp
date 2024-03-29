/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NDT_2D__NDT_MAPPER_HPP_
#define NDT_2D__NDT_MAPPER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ndt_2d/ceres_solver.hpp>
#include <ndt_2d/graph.hpp>
#include <ndt_2d/occupancy_grid.hpp>
#include <ndt_2d/particle_filter.hpp>
#include <ndt_2d/scan_matcher.hpp>
#include <ndt_2d/srv/configure.hpp>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ndt_2d
{

class Mapper : public rclcpp::Node
{
public:
  explicit Mapper(const rclcpp::NodeOptions & options);
  virtual ~Mapper();

protected:
  /** @brief ROS callback for configure service */
  void configure(const std::shared_ptr<ndt_2d::srv::Configure::Request> request,
                 std::shared_ptr<ndt_2d::srv::Configure::Response> response);

  /** @brief ROS callback for initializing localization */
  void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);

  /** @brief ROS callback for new laser scan */
  void laserCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);

  // Thread for doing global loop closure
  void loopClosureThread();
  size_t global_scans_processed_;
  std::unique_ptr<std::thread> loop_closure_thread_;

  // Thread for publishing map and transforms
  void mapPublishThread();
  bool map_update_available_;
  std::unique_ptr<std::thread> map_publish_thread_;

  // Mapping parameters
  double map_resolution_;
  double minimum_travel_distance_, minimum_travel_rotation_;
  size_t rolling_depth_;
  std::string odom_frame_, robot_frame_, laser_frame_;
  bool laser_inverted_;
  bool use_barycenter_;
  double global_search_size_;
  size_t global_search_limit_;
  // How many nodes need to be added between optimization
  size_t optimization_node_limit_, optimization_last_;
  double transform_timeout_;
  double range_max_;

  // Localization parameters
  bool use_particle_filter_;
  double kld_err_, kld_z_;
  std::shared_ptr<ParticleFilter> filter_;

  // Scan matchers
  // Used for loop closure when mapping, and localization
  ScanMatcherPtr global_scan_matcher_;
  // Used for odometric correction when mapping
  ScanMatcherPtr local_scan_matcher_;
  pluginlib::ClassLoader<ScanMatcher> scan_matcher_loader_;
  std::string scan_matcher_type_;
  double typical_matcher_response_;

  // ROS 2 interfaces
  rclcpp::Logger logger_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr particle_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Service<ndt_2d::srv::Configure>::SharedPtr configure_srv_;
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;

  // Map data
  GraphPtr graph_;
  std::mutex graph_mutex_;
  bool enable_mapping_;
  std::mutex prev_pose_mutex_;
  // The previous odom_frame_->robot_frame_
  Pose2d prev_odom_pose_;
  bool prev_odom_pose_is_initialized_;
  // The previous map->robot_frame_
  Pose2d prev_robot_pose_;
  // Transform from robot_frame_->laser_frame_
  Pose2d laser_transform_;

  // Graph optimization
  std::shared_ptr<CeresSolver> solver_;

  // Map export
  OccupancyGridPtr grid_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__NDT_MAPPER_HPP_
