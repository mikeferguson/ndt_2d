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

#include <iostream>

#include <cmath>
#include <ndt_2d/occupancy_grid.hpp>

namespace ndt_2d
{
OccupancyGrid::OccupancyGrid(const double resolution, const double occ_thresh)
: resolution_(resolution),
  occ_thresh_(occ_thresh),
  min_x_(0),
  max_x_(0),
  min_y_(0),
  max_y_(0),
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
  grid.data.assign(grid.info.width * grid.info.height, -1);

  // Create working arrays
  std::vector<int> hit(grid.data.size(), 0);
  std::vector<int> empty(grid.data.size(), 0);

  // Render scans into grid
  for (auto & scan : scans)
  {
    double pose_x = scan->getPose().x;
    double pose_y = scan->getPose().y;
    double cos_th = cos(scan->getPose().theta);
    double sin_th = sin(scan->getPose().theta);

    // Start raytracing each line from the pose
    const int start_x = (pose_x - grid.info.origin.position.x) / resolution_;
    const int start_y = (pose_y - grid.info.origin.position.y) / resolution_;

    for (auto & point : scan->getPoints())
    {
      const double point_x = point.x * cos_th - point.y * sin_th + pose_x;
      const double point_y = point.x * sin_th + point.y * cos_th + pose_y;

      const int end_x = (point_x - grid.info.origin.position.x) / resolution_;
      const int end_y = (point_y - grid.info.origin.position.y) / resolution_;

      // Simplified Bresenham
      int dx = abs(end_x - start_x);
      int sx = (start_x < end_x) ? 1 : -1;
      int dy = -abs(end_y - start_y);
      int sy = (start_y < end_y) ? 1 : -1;
      int error = dx + dy;

      int x = start_x;
      int y = start_y;

      while (true)
      {
        int index = x + y * grid.info.width;
        if (x == end_x && y == end_y)
        {
          ++hit[index];
          break;
        }
        ++empty[index];
        if (2 * error >= dy)
        {
          if (x == end_x)
          {
            ++hit[index];
            break;
          }
          error = error + dy;
          x += sx;
        }
        if (2 * error <= dx)
        {
          if (y == end_y)
          {
            ++hit[index];
            break;
          }
          error = error + dx;
          y += sy;
        }
      }
    }
  }

  // Finalize the map
  for (size_t i = 0; i < grid.data.size(); ++i)
  {
    double touches = hit[i] + empty[i];
    if (touches > 0.5)
    {
      // We have information about this cell
      if (static_cast<double>(hit[i]) / touches > occ_thresh_)
      {
        grid.data[i] = 100;
      }
      else
      {
        grid.data[i] = 0;
      }
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
    double x = scans[i]->getPose().x;
    double y = scans[i]->getPose().y;
    double cos_th = cos(scans[i]->getPose().theta);
    double sin_th = sin(scans[i]->getPose().theta);

    // Iterate through points in the scan
    for (auto & point : scans[i]->getPoints())
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
