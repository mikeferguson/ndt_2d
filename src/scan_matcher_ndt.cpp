/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <Eigen/Core>
#include <ndt_2d/ndt_mapper.hpp>
#include <ndt_2d/scan_matcher_ndt.hpp>

namespace ndt_2d
{

void ScanMatcherNDT::initialize(rclcpp::Node * node, double range_max)
{
  resolution_ = node->declare_parameter<double>("ndt_resolution", 0.25);

  angular_res_ = node->declare_parameter<double>("search_angular_resolution", 0.0025);
  angular_size_ = node->declare_parameter<double>("search_angular_size", 0.1);
  linear_res_ = node->declare_parameter<double>("search_linear_resolution", 0.005);
  linear_size_ = node->declare_parameter<double>("search_linear_size", 0.05);

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
    min_x_ = std::min((*scan)->pose.x - range_max_, min_x_);
    max_x_ = std::max((*scan)->pose.x + range_max_, max_x_);
    min_y_ = std::min((*scan)->pose.y - range_max_, min_y_);
    max_y_ = std::max((*scan)->pose.y + range_max_, max_y_);
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
                                 Eigen::Matrix3d & covariance,
                                 size_t scan_points_to_use) const
{
  // Scans must be added first
  if (!ndt_) return 0.0;

  // Search NDT for best correlation for new scan
  double best_score = 0;

  // Working values for covariance computation
  Eigen::Matrix3d k = Eigen::Matrix3d::Zero();
  Eigen::Vector3d u = Eigen::Vector3d::Zero();
  double s = 0.0;

  // Subsample the scan
  scan_points_to_use = std::min(scan_points_to_use, scan->points.size());
  double scan_step = static_cast<double>(scan->points.size()) / scan_points_to_use;

  std::vector<Point> points_outer;
  std::vector<Point> points_inner;
  points_outer.resize(scan_points_to_use);
  points_inner.resize(scan_points_to_use);

  for (double dth = -angular_size_; dth < angular_size_; dth += angular_res_)
  {
    // Do orientation on the outer loop - then we can simply shift points in inner loops
    double costh = cos(scan->pose.theta + dth);
    double sinth = sin(scan->pose.theta + dth);
    for (size_t i = 0; i < points_outer.size(); ++i)
    {
      size_t scan_idx = static_cast<size_t>(i * scan_step);
      points_outer[i].x = scan->points[scan_idx].x * costh -
                          scan->points[scan_idx].y * sinth + scan->pose.x;
      points_outer[i].y = scan->points[scan_idx].x * sinth +
                          scan->points[scan_idx].y * costh + scan->pose.y;
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

        double likelihood = -ndt_->likelihood(points_inner);
        if (likelihood < best_score)
        {
          best_score = likelihood;
          pose.x = dx;
          pose.y = dy;
          pose.theta = dth;
        }

        // Covariance computation
        Eigen::Vector3d x(dx, dy, dth);
        k += x * x.transpose() * likelihood;
        u += x * likelihood;
        s += likelihood;
      }
    }
  }

  // Compute covariance
  covariance = (1 / s) * k + (1 / (s * s) * u * u.transpose());

  return best_score;
}

double ScanMatcherNDT::scoreScan(const ScanPtr & scan) const
{
  if (!ndt_) return 0.0;
  return -ndt_->likelihood(scan);
}

double ScanMatcherNDT::scoreScan(const ScanPtr & scan, const Pose2d & pose) const
{
  if (!ndt_) return 0.0;
  return -ndt_->likelihood(scan, pose);
}

double ScanMatcherNDT::scorePoints(const std::vector<Point> & points, const Pose2d & pose) const
{
  if (!ndt_) return 0.0;
  return -ndt_->likelihood(points, pose);
}

void ScanMatcherNDT::reset()
{
  ndt_.reset();
}

}  // namespace ndt_2d
