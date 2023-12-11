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
      const Pose2d & pose = use_barycenter_ ? scan->getBarycenterPose() : scan->getPose();
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
