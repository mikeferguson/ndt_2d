/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__OCCUPANCY_GRID_HPP_
#define NDT_2D__OCCUPANCY_GRID_HPP_

#include <memory>
#include <vector>
#include <ndt_2d/ndt_model.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace ndt_2d
{
class OccupancyGrid
{
public:
  /**
   * @brief Create an occupancy grid generator.
   * @param resolution The map resolution to export.
   */
  explicit OccupancyGrid(double resolution);

  /**
   * @brief Create an occupancy grid message from a series of scans.
   * @param scans The scans to render into the map.
   * @param grid The map message to be filled with data.
   */
  void getMsg(std::vector<ndt_2d::ScanPtr> & scans,
              nav_msgs::msg::OccupancyGrid & grid);

private:
  void updateBounds(std::vector<ndt_2d::ScanPtr>& scans);

  double resolution_;
  // Bounds are recalculated when scan vector increases in size
  double min_x_, max_x_, min_y_, max_y_;
  size_t num_scans_;
};

typedef std::shared_ptr<OccupancyGrid> OccupancyGridPtr;

}  // namespace ndt_2d

#endif  // NDT_2D__OCCUPANCY_GRID_HPP_
