/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
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
