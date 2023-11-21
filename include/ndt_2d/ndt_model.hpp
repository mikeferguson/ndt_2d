/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__NDT_MODEL_HPP_
#define NDT_2D__NDT_MODEL_HPP_

#include <Eigen/Core>
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

  double x, y, theta;
};

struct Scan
{
  std::vector<Point> points;
  Pose2d pose;
};
typedef std::shared_ptr<Scan> ScanPtr;

struct Cell
{
  using vector_t = Eigen::Matrix<double, 2, 1>;
  using matrix_t = Eigen::Matrix<double, 2, 2>;

  explicit Cell();

  /** @brief Add a point to this cell */
  void addPoint(Point & p);

  /** @brief Compute the cell values */
  void compute();

  /** @brief Score a point */
  double score(Point & p);

  // Are mean/cov valid;
  bool valid;

  // Technically this should be size_t - but for Eigen math, needs to be double
  double n;
  vector_t mean;
  matrix_t covariance;
  matrix_t correlation;
  matrix_t information;
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
   * @param pose Pose to use (we ignore the scan pose).
   */
  void addScan(ScanPtr& scan, Pose2d& pose);

  /**
   * @brief Compute NDT cell values - this must be called after any
   *        scans are added before you can query the cells.
   */
  void compute();

  /**
   * @brief Query the NDT.
   * @param points The vector of points to score.
   * @returns The probability of the points.
   */
  double likelihood(std::vector<Point>& points);

  /**
   * @brief Query the NDT. Typically used to create occupancy grid.
   * @param points The vector of points to score.
   * @param scores The score for each point.
   */
  void likelihood(std::vector<Point>& points, std::vector<double>& scores);

  /**
   * @brief Query the NDT.
   * @param point The point to score.
   * @returns The probability of the point.
   */
  double likelihood(Point& point);

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
