/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__GRAPH_HPP_
#define NDT_2D__GRAPH_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <string>
#include <nanoflann.hpp>
#include <ndt_2d/constraint.hpp>
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
  explicit Graph(bool use_barycenter);
  ~Graph();

  /**
   * @brief Create a graph by loading from a file
   * @param filename Full path to the map file
   */
  Graph(bool use_barycenter, const std::string & filename);

  /**
   * @brief Save all scans and constraints to a file
   */
  bool save(const std::string & filename);

  /**
   * @brief Find the index of the nearest scan to this scan
   * @param scan Find the nearest indices to scan->pose
   * @param dist Max distance to search
   * @param limit_scan_index Limit of scans to search.
   */
  std::vector<size_t> findNearest(const ScanPtr & scan, double dist = 10.0,
                                  int limit_scan_index = -1);

  /**
   * @brief Get a visualization msg for the graph
   */
  void getMsg(visualization_msgs::msg::MarkerArray & msg, rclcpp::Time & t);

  // Vector of scans used to build the map
  std::vector<ScanPtr> scans;
  // Constraints between scans
  std::vector<ConstraintPtr> constraints;

private:
  // Support for nanoflann
  struct GraphAdapter
  {
    explicit GraphAdapter(Graph * graph, size_t size, bool use_barycenter)
    : graph_(graph),
      size_(size),
      use_barycenter_(use_barycenter)
    {
    }

    inline size_t kdtree_get_point_count() const { return size_; }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const
    {
      const ScanPtr & scan = graph_->scans[idx];
      const Pose2d & pose = use_barycenter_ ? scan->barycenter : scan->pose;
      if (dim == 0) return pose.x;
      else return pose.y;
    }

    // Use the standard bounding-box computations
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }

    Graph * graph_;
    size_t size_;
    bool use_barycenter_;
  };

  using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, GraphAdapter>, GraphAdapter, 2>;

  bool use_barycenter_;
};

using GraphPtr = std::shared_ptr<Graph>;

}  // namespace ndt_2d

#endif  // NDT_2D__GRAPH_HPP_
