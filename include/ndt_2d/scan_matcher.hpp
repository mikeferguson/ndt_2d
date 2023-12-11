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

#ifndef NDT_2D__SCAN_MATCHER_HPP_
#define NDT_2D__SCAN_MATCHER_HPP_

#include <Eigen/Core>
#include <memory>
#include <string>
#include <vector>
#include <ndt_2d/scan.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ndt_2d
{

class ScanMatcher
{
public:
  virtual ~ScanMatcher() = default;

  /**
   * @brief Initialize a scan matcher instance.
   * @param name Name for ths scan matcher instance.
   * @param node Node instance to use for getting parameters.
   * @param range_max Maximum range of laser scanner.
   */
  virtual void initialize(const std::string & name,
                          rclcpp::Node * node, double range_max) = 0;

  /**
   * @brief Add scans to the internal map.
   * @param begin Starting iterator of scans for map building.
   * @param end Ending iterator of scans for map building.
   */
  virtual void addScans(const std::vector<ScanPtr>::const_iterator & begin,
                        const std::vector<ScanPtr>::const_iterator & end) = 0;

  /**
   * @brief Match a scan against the internal map.
   * @param scan Scan to match against internal map.
   * @param pose The corrected pose that best matches scan to map.
   * @param covariance Covariance matrix for the match.
   * @returns The score when scan is at corrected pose.
   */
  virtual double matchScan(const ScanPtr & scan, Pose2d & pose,
                           Eigen::Matrix3d & covariance) const = 0;

  /**
   * @brief Score a scan against the internal map.
   * @param scan Scan to score against internal map.
   */
  virtual double scoreScan(const ScanPtr & scan) const = 0;

  /**
   * @brief Score a set of points against the internal map.
   * @param points Points to score against internal map.
   * @param pose The pose of the points within the internal map.
   */
  virtual double scorePoints(const std::vector<Point> & points, const Pose2d & pose) const = 0;

  /**
   * @brief Reset the internal map, removing all scans.
   */
  virtual void reset() = 0;
};

using ScanMatcherPtr = std::shared_ptr<ScanMatcher>;

}  // namespace ndt_2d

#endif  // NDT_2D__SCAN_MATCHER_HPP_
