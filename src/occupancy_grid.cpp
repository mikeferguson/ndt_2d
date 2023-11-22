/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>

#include <cmath>
#include <ndt_2d/occupancy_grid.hpp>

namespace ndt_2d
{
OccupancyGrid::OccupancyGrid(double resolution)
: resolution_(resolution),
  min_x_(-1.0),
  max_x_(1.0),
  min_y_(-1.0),
  max_y_(1.0),
  num_scans_(0)
{
}

void OccupancyGrid::getMsg(std::vector<ndt_2d::ScanPtr> & scans,
                           nav_msgs::msg::OccupancyGrid & grid)
{
  // Update size of the occupancy grid, if needed
  if (scans.size() != num_scans_)
  {
    updateBounds(scans);
  }

  // Pad the map just a bit
  double pad = 5 * resolution_;

  // Setup meta data
  grid.info.resolution = resolution_;
  grid.info.width = (max_x_ - min_x_ + 2 * pad) / resolution_;
  grid.info.height = (max_y_ - min_y_ + 2 * pad) / resolution_;
  grid.info.origin.position.x = min_x_ - pad;
  grid.info.origin.position.y = min_y_ - pad;
  grid.info.origin.orientation.w = 1.0;
  // TODO(fergs): initialize to -1 once we do raytracing
  grid.data.assign(grid.info.width * grid.info.height, 0);

  // Create working arrays
  std::vector<int> hit(grid.data.size(), 0);

  // Render scans into grid
  for (auto & scan : scans)
  {
    double x = scan->pose.x;
    double y = scan->pose.y;
    double cos_th = cos(scan->pose.theta);
    double sin_th = sin(scan->pose.theta);

    for (auto & point : scan->points)
    {
      Point p(x, y);
      p.x += point.x * cos_th - point.y * sin_th;
      p.y += point.x * sin_th + point.y * cos_th;

      int idx = (p.x - grid.info.origin.position.x) / resolution_;
      int idy = (p.y - grid.info.origin.position.y) / resolution_;
      int index = idx + idy * grid.info.width;
      // TODO(fergs): raytrace and threshold
      ++hit[index];
    }
  }

  // Finalize the map
  for (size_t i = 0; i < grid.data.size(); ++i)
  {
    if (hit[i] >= 1)
    {
      grid.data[i] = 100;
    }
  }
}

void OccupancyGrid::updateBounds(std::vector<ndt_2d::ScanPtr>& scans)
{
  size_t start_idx = num_scans_;
  num_scans_ = scans.size();

  // Iterate through new scans
  for (size_t i = start_idx; i < num_scans_; ++i)
  {
    double x = scans[i]->pose.x;
    double y = scans[i]->pose.y;
    double cos_th = cos(scans[i]->pose.theta);
    double sin_th = sin(scans[i]->pose.theta);

    // Iterate through points in the scan
    for (auto & point : scans[i]->points)
    {
      Point p(x, y);
      p.x += point.x * cos_th - point.y * sin_th;
      p.y += point.x * sin_th + point.y * cos_th;
      min_x_ = std::min(p.x, min_x_);
      max_x_ = std::max(p.x, max_x_);
      min_y_ = std::min(p.y, min_y_);
      max_y_ = std::max(p.y, max_y_);
    }
  }

  // Make min/max values a multiple of resolution
  min_x_ = std::floor(min_x_ / resolution_) * resolution_;
  max_x_ = std::ceil(max_x_ / resolution_) * resolution_;
  min_y_ = std::floor(min_y_ / resolution_) * resolution_;
  max_y_ = std::ceil(max_y_ / resolution_) * resolution_;
}

}  // namespace ndt_2d
