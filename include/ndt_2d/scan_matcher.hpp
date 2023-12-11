/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
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
