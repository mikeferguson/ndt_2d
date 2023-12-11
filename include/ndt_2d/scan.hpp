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

#ifndef NDT_2D__SCAN_HPP_
#define NDT_2D__SCAN_HPP_

#include <memory>
#include <vector>
#include <ndt_2d/point.hpp>
#include <ndt_2d/pose_2d.hpp>

namespace ndt_2d
{

class Scan
{
public:
  /**
   * @brief Create a new scan, with given id.
   * @param id The unique ID of this scan.
   */
  explicit Scan(size_t id);

  /**
   * @brief Get the scan id.
   */
  size_t getId();

  /**
   * @brief Set the pose of the scan.
   * @param pose The pose of the scan.
   */
  void setPose(const Pose2d & pose);

  /**
   * @brief Get the pose of the scan.
   */
  Pose2d getPose();

  /**
   * @brief Get the barycenter pose of the scan.
   */
  Pose2d getBarycenterPose();

  /**
   * @brief Set the scan data.
   */
  void setPoints(const std::vector<Point> & points);

  /**
   * @brief Get the scan data.
   */
  std::vector<Point> getPoints();

private:
  void update();

  size_t id_;
  bool dirty_;
  Pose2d pose_;
  Pose2d barycenter_;
  std::vector<Point> points_;
};

typedef std::shared_ptr<Scan> ScanPtr;

}  // namespace ndt_2d

#endif  // NDT_2D__SCAN_HPP_
