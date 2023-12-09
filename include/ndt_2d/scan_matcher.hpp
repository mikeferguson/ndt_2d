/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__SCAN_MATCHER_HPP_
#define NDT_2D__SCAN_MATCHER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace ndt_2d
{

class ScanMatcher
{
public:
  virtual ~ScanMatcher() = default;

  /**
   * @brief Initialize an NDT scan matcher instance.
   * @param name Name for ths scan matcher instance.
   * @param node Node instance to use for getting parameters.
   * @param range_max Maximum range of laser scanner.
   */
  virtual void initialize(const std::string & name,
                          rclcpp::Node * node, double range_max) = 0;

  /**
   * @brief Add scans to the internal NDT map.
   * @param begin Starting iterator of scans for NDT map building.
   * @param end Ending iterator of scans for NDT map building.
   */
  virtual void addScans(const std::vector<ScanPtr>::const_iterator & begin,
                        const std::vector<ScanPtr>::const_iterator & end) = 0;

  /**
   * @brief Match a scan against the internal NDT map.
   * @param scan Scan to match against internal NDT map.
   * @param pose The corrected pose that best matches scan to NDT map.
   * @param covariance Covariance matrix for the match.
   * @param scan_points_to_use Number of points to match from the scan.
   * @returns The likelihood score when scan is at corrected pose.
   */
  virtual double matchScan(const ScanPtr & scan, Pose2d & pose,
                           Eigen::Matrix3d & covariance,
                           size_t scan_points_to_use) const = 0;

  /**
   * @brief Score a scan against the internal NDT map.
   * @param scan Scan to score against internal NDT map.
   */
  virtual double scoreScan(const ScanPtr & scan) const = 0;

  /**
   * @brief Score a scan against the internal NDT map.
   * @param scan Scan to score against internal NDT map.
   * @param pose The pose of the scan within the internal NDT map.
   */
  virtual double scoreScan(const ScanPtr & scan, const Pose2d & pose) const = 0;

  /**
   * @brief Score a set of points against the internal NDT map.
   * @param points Points to score against internal NDT map.
   * @param pose The pose of the points within the internal NDT map.
   */
  virtual double scorePoints(const std::vector<Point> & points, const Pose2d & pose) const = 0;

  /**
   * @brief Reset the internal NDT map, removing all scans.
   */
  virtual void reset() = 0;
};

using ScanMatcherPtr = std::shared_ptr<ScanMatcher>;

}  // namespace ndt_2d

#endif  // NDT_2D__SCAN_MATCHER_HPP_
