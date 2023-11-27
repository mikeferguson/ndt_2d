/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__NDT_MODEL_HPP_
#define NDT_2D__NDT_MODEL_HPP_

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

namespace ndt_2d
{

struct Point
{
  Point()
  {
    x = 0;
    y = 0;
  }

  Point(double x, double y)
  {
    this->x = x;
    this->y = y;
  }

  // Point location, in meters
  double x, y;
};

struct Pose2d
{
  Pose2d()
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
  }

  Pose2d(double x, double y, double theta)
  {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }

  // Pose position in meters
  double x, y;
  // Pose orientation in radians
  double theta;
};

struct Scan
{
  // Unique ID of the scan
  size_t id;
  // Pose of scan in the "map" frame
  Pose2d pose;
  // Points that form the laser scan
  std::vector<Point> points;
};
typedef std::shared_ptr<Scan> ScanPtr;

struct Constraint
{
  size_t begin;
  size_t end;
  // dx, dy, dtheta between begin and end poses
  Eigen::Vector3d transform;
  Eigen::Matrix3d information;
};
typedef std::shared_ptr<Constraint> ConstraintPtr;

struct Cell
{
  Cell();

  /** @brief Add a point to this cell */
  void addPoint(const Eigen::Vector2d & p);

  /** @brief Compute the cell values */
  void compute();

  /** @brief Score a point */
  double score(const Eigen::Vector2d & p);

  // Are mean/cov valid;
  bool valid;

  // Technically this should be size_t - but for Eigen math, needs to be double
  double n;
  Eigen::Vector2d mean;
  Eigen::Matrix2d covariance;
  Eigen::Matrix2d correlation;
  Eigen::Matrix2d information;
};

class NDT
{
public:
  /**
   * @brief Create an instance of an NDT distribution.
   * @param cell_size Size of NDT cells in meters.
   * @param size_x Size of NDT in meters.
   * @param size_y Size of NDT in meters.
   * @param origin_x Coordinate of lower left corner in meters.
   * @param origin_y Coordinate of lower left corner in meters.
   */
  NDT(double cell_size, double size_x, double size_y, double origin_x, double origin_y);

  virtual ~NDT();

  /**
   * @brief Add a scan to the NDT.
   * @param scan The points from laser scanner to be added.
   */
  void addScan(const ScanPtr & scan);

  /**
   * @brief Compute NDT cell values - this must be called after any
   *        scans are added before you can query the cells.
   */
  void compute();

  /**
   * @brief Query the NDT for a given point.
   * @param point The point to score.
   * @returns The probability of the point.
   */
  double likelihood(const Eigen::Vector2d & point);

  /**
   * @brief Query the NDT for a given point.
   * @param point The point to score.
   * @returns The probability of the point.
   */
  double likelihood(const Eigen::Vector3d & point);

  /**
   * @brief Query the NDT.
   * @param points The vector of points to score.
   * @returns The probability of the points.
   */
  double likelihood(const std::vector<Point> & points);

  /**
   * @brief Query the NDT.
   * @param points The vector of points to score.
   * @param pose All points will be translated as if this were the origin of the points.
   * @returns The probability of the points.
   */
  double likelihood(const std::vector<Point> & points, const Pose2d & pose);

  /**
   * @brief Query the NDT.
   * @param scan The scan to score. Note that scan->pose WILL be used.
   * @returns The probability of the scan.
   */
  double likelihood(const ScanPtr & scan);

  /**
   * @brief Query the NDT.
   * @param scan The scan to score. Note that scan->pose WILL be used.
   * @param correction Pose correction to add to scan pose.
   * @returns The probability of the scan.
   */
  double likelihood(const ScanPtr & scan, const Pose2d & correction);

private:
  /**
   * @brief Get the index of a cell within cells_
   * @param x The x coordinate (in meters).
   * @param y The y coordinate (in meters).
   */
  int getIndex(double x, double y);

  double cell_size_;
  size_t size_x_, size_y_;
  double origin_x_, origin_y_;
  std::vector<Cell> cells_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__NDT_MODEL_HPP_
