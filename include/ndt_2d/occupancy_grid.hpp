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
   * @param occ_thresh Occupied threshold.
   */
  explicit OccupancyGrid(const double resolution,
                         const double occ_thresh);

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
  double occ_thresh_;
  // Bounds are recalculated when scan vector increases in size
  double min_x_, max_x_, min_y_, max_y_;
  size_t num_scans_;
};

typedef std::shared_ptr<OccupancyGrid> OccupancyGridPtr;

}  // namespace ndt_2d

#endif  // NDT_2D__OCCUPANCY_GRID_HPP_
