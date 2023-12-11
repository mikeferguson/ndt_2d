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

#include <ndt_2d/conversions.hpp>
#include <ndt_2d/scan_matcher_ndt.hpp>

namespace ndt_2d
{

void ScanMatcherNDT::initialize(const std::string & name, rclcpp::Node * node, double range_max)
{
  resolution_ = node->declare_parameter<double>(name + ".ndt_resolution", 0.25);

  angular_res_ = node->declare_parameter<double>(name + ".search_angular_resolution", 0.0025);
  angular_size_ = node->declare_parameter<double>(name + ".search_angular_size", 0.1);
  linear_res_ = node->declare_parameter<double>(name + ".search_linear_resolution", 0.005);
  linear_size_ = node->declare_parameter<double>(name + ".search_linear_size", 0.05);

  laser_max_beams_ = node->declare_parameter<int>(name + ".laser_max_beams", 100);

  range_max_ = range_max;
}

void ScanMatcherNDT::addScans(const std::vector<ScanPtr>::const_iterator& begin,
                              const std::vector<ScanPtr>::const_iterator& end)
{
  // Compute bounding box required
  double min_x_ = std::numeric_limits<double>::max();
  double max_x_ = std::numeric_limits<double>::min();
  double min_y_ = std::numeric_limits<double>::max();
  double max_y_ = std::numeric_limits<double>::min();
  for (auto scan = begin; scan != end; ++scan)
  {
    Pose2d pose = (*scan)->getPose();
    min_x_ = std::min(pose.x - range_max_, min_x_);
    max_x_ = std::max(pose.x + range_max_, max_x_);
    min_y_ = std::min(pose.y - range_max_, min_y_);
    max_y_ = std::max(pose.y + range_max_, max_y_);
  }

  ndt_ = std::make_unique<NDT>(resolution_,
                               (max_x_ - min_x_), (max_y_ - min_y_), min_x_, min_y_);
  for (auto scan = begin; scan != end; ++scan)
  {
    ndt_->addScan(*scan);
  }

  ndt_->compute();
}

double ScanMatcherNDT::matchScan(const ScanPtr & scan, Pose2d & pose,
                                 Eigen::Matrix3d & covariance) const
{
  // Scans must be added first
  if (!ndt_) return 0.0;

  // Search NDT for best correlation for new scan
  double best_score = 0;

  // Working values for covariance computation
  Eigen::Matrix3d k = Eigen::Matrix3d::Zero();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  double s = 0.0;

  // Local copies
  Pose2d scan_pose = scan->getPose();
  std::vector<Point> points = scan->getPoints();

  // Subsample the scan
  size_t scan_points_to_use = std::min(laser_max_beams_, points.size());
  double scan_step = static_cast<double>(points.size()) / scan_points_to_use;

  std::vector<Point> points_outer;
  std::vector<Point> points_inner;
  points_outer.resize(scan_points_to_use);
  points_inner.resize(scan_points_to_use);

  for (double dth = -angular_size_; dth < angular_size_; dth += angular_res_)
  {
    // Do orientation on the outer loop - then we can simply shift points in inner loops
    double costh = cos(scan_pose.theta + dth);
    double sinth = sin(scan_pose.theta + dth);
    for (size_t i = 0; i < points_outer.size(); ++i)
    {
      size_t scan_idx = static_cast<size_t>(i * scan_step);
      points_outer[i].x = points[scan_idx].x * costh -
                          points[scan_idx].y * sinth + scan_pose.x;
      points_outer[i].y = points[scan_idx].x * sinth +
                          points[scan_idx].y * costh + scan_pose.y;
    }

    for (double dx = -linear_size_; dx < linear_size_; dx += linear_res_)
    {
      for (double dy = -linear_size_; dy < linear_size_; dy += linear_res_)
      {
        for (size_t i = 0; i < points_inner.size(); ++i)
        {
          points_inner[i].x = points_outer[i].x + dx;
          points_inner[i].y = points_outer[i].y + dy;
        }

        double score = -ndt_->likelihood(points_inner);
        if (score < best_score)
        {
          best_score = score;
          pose.x = dx;
          pose.y = dy;
          pose.theta = dth;
        }

        // Covariance computation
        Eigen::Vector3d x(dx, dy, dth);
        k += x * x.transpose() * score;
        u += x * score;
        s += score;
      }
    }
  }

  // Compute covariance
  covariance = (1 / s) * k + (1 / (s * s) * u * u.transpose());

  return best_score / scan_points_to_use;
}

double ScanMatcherNDT::scoreScan(const ScanPtr & scan) const
{
  return scorePoints(scan->getPoints(), scan->getPose());
}

double ScanMatcherNDT::scorePoints(const std::vector<Point> & points, const Pose2d & pose) const
{
  // Need a valid NDT
  if (!ndt_) return 0.0;

  // Transform points to the pose
  const Eigen::Isometry3d t = toEigen(pose);

  // Subsample the scan
  size_t scan_points_to_use = std::min(laser_max_beams_, points.size());
  double scan_step = static_cast<double>(points.size()) / scan_points_to_use;

  double score = 0.0;
  for (size_t i = 0; i < scan_points_to_use; ++i)
  {
    size_t scan_idx = static_cast<size_t>(i * scan_step);
    Eigen::Vector3d p(points[scan_idx].x, points[scan_idx].y, 1.0);
    p = t * p;
    score += -ndt_->likelihood(p);
  }

  return score / scan_points_to_use;
}

void ScanMatcherNDT::reset()
{
  ndt_.reset();
}

}  // namespace ndt_2d

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ndt_2d::ScanMatcherNDT, ndt_2d::ScanMatcher)
