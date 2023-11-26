/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__GRAPH_HPP_
#define NDT_2D__GRAPH_HPP_

#include <memory>
#include <vector>
#include <string>
#include <ndt_2d/ndt_model.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace ndt_2d
{

class Graph
{
public:
  /**
   * @brief Create an empty graph
   */
  Graph();
  ~Graph();

  /**
   * @brief Load a graph from a file
   */
  explicit Graph(const std::string & filename);

  /**
   * @brief Save all scans and constraints to a file
   */
  bool save(const std::string & filename);

  /**
   * @brief Get a visualization msg for the graph
   */
  void getMsg(visualization_msgs::msg::MarkerArray & msg, rclcpp::Time & t);

  // Vector of scans used to build the map
  std::vector<ScanPtr> scans;
  // Odometry constraints between consecutive scans
  std::vector<ConstraintPtr> odom_constraints;
  // Loop closure constraints between scans
  std::vector<ConstraintPtr> loop_constraints;
};

using GraphPtr = std::shared_ptr<Graph>;

}  // namespace ndt_2d

#endif  // NDT_2D__GRAPH_HPP_
