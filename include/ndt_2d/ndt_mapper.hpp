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
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <ndt_2d/ceres_solver.hpp>
#include <ndt_2d/graph.hpp>
#include <ndt_2d/occupancy_grid.hpp>
#include <ndt_2d/particle_filter.hpp>
#include <ndt_2d/srv/configure.hpp>
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

  /**
   * @brief Build an NDT map from a set of scans.
   * @param begin Starting iterator of scans for NDT map building.
   * @param end Ending iterator of scans for NDT map building.
   * @param ndt Shared pointer to the NDT map built.
   */
  void buildNDT(const std::vector<ScanPtr>::const_iterator & begin,
                const std::vector<ScanPtr>::const_iterator & end,
                std::shared_ptr<NDT> & ndt);

  /**
   * @brief Match a scan against an NDT map.
   * @param ndt Map to match scan against.
   * @param scan Scan to match against NDT map.
   * @param pose The corrected pose that best matches scan to NDT map.
   * @param scan_points_to_use Number of points to match from the scan.
   * @returns The likelihood score when scan is at corrected pose.
   */
  double matchScan(const std::shared_ptr<NDT> & ndt,
                   const ScanPtr & scan, Pose2d & pose,
                   size_t scan_points_to_use);

  void searchGlobalMatches(ScanPtr & scan);
  void publishTransform();
  void mapPublishCallback();
  bool map_update_available_;

  // Mapping parameters
  double map_resolution_;
  double ndt_resolution_;
  double minimum_travel_distance_, minimum_travel_rotation_;
  size_t rolling_depth_;
  std::string odom_frame_, robot_frame_, laser_frame_;
  bool laser_inverted_;
  size_t laser_max_beams_;
  double search_angular_resolution_, search_angular_size_;
  double search_linear_resolution_, search_linear_size_;
  double global_search_size_;
  double transform_timeout_;
  double range_max_;

  // Localization parameters
  bool use_particle_filter_;
  double kld_err_, kld_z_;
  std::shared_ptr<ParticleFilter> filter_;
  std::shared_ptr<NDT> global_ndt_;

  // ROS 2 interfaces
  rclcpp::Logger logger_;
  rclcpp::TimerBase::SharedPtr map_publish_timer_;
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
  bool enable_mapping_;
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
