/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <iostream>

#include <cmath>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

Cell::Cell()
: valid(false),
  n(0),
  mean(vector_t::Zero()),
  covariance(matrix_t::Zero()),
  correlation(matrix_t::Zero()),
  information(matrix_t::Zero())
{
}

void Cell::addPoint(const Point & point)
{
  // TODO(fergs): take Eigen type directly
  vector_t p;
  p(0) = point.x;
  p(1) = point.y;

  mean = (mean * n + p) / (n + 1);
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = i; j < 3; ++j)
    {
      correlation(i, j) = (correlation(i, j) * n + p(i) * p(j)) / (n + 1);
    }
  }

  n += 1;
  valid = false;
}

void Cell::compute()
{
  // No need to update
  if (valid || n < 3)
  {
    return;
  }

  const double scale = n / (n - 1);
  for (size_t i = 0; i < 3; ++i)
  {
    for (size_t j = 0; j < 3; ++j)
    {
      covariance(i, j) = (correlation(i, j) - (mean(i) * mean(j))) * scale;
      covariance(j, i) = correlation(i, j);
    }
  }

  // TODO(fergs): limit eigen values

  information = covariance.inverse();
  valid = true;
}

double Cell::score(const Point & point)
{
  if (n < 3)
  {
    // Need at least three points for our mean/cov to be valid
    return 0.0;
  }

  // TODO(fergs): take Eigen type directly
  vector_t p;
  p(0) = point.x;
  p(1) = point.y;

  const auto q = p - mean;
  const double exponent = -0.5 * q.transpose() * information * q;
  return std::exp(exponent);
}

NDT::NDT(double cell_size, double size_x, double size_y, double origin_x, double origin_y)
{
  cell_size_ = cell_size;
  size_x_ = (size_x / cell_size_) + 1;
  size_y_ = (size_y / cell_size_) + 1;
  origin_x_ = origin_x;
  origin_y_ = origin_y;
  cells_.resize(size_x_ * size_y_);
  std::cout << "Created an NDT of " << size_x_ << " by " << size_y_ << " cells" << std::endl;
}

NDT::~NDT()
{
}

void NDT::addScan(const ScanPtr& scan)
{
  // Precompute transforms
  double cos_th = cos(scan->pose.theta);
  double sin_th = sin(scan->pose.theta);

  for (auto & point : scan->points)
  {
    // Transform the point by pose
    Point p(scan->pose.x, scan->pose.y);
    p.x += point.x * cos_th - point.y * sin_th;
    p.y += point.x * sin_th + point.y * cos_th;

    // Determine index in NDT grid, add if valid index
    int index = getIndex(p.x, p.y);
    if (index >= 0)
    {
      cells_[index].addPoint(p);
    }
  }
}

void NDT::compute()
{
  for (auto & cell : cells_)
  {
    cell.compute();
  }
}

double NDT::likelihood(const std::vector<Point>& points)
{
  double score = 0.0;
  for (auto & point : points)
  {
    score += likelihood(point);
  }
  return score;
}

void NDT::likelihood(const std::vector<Point>& points, std::vector<double>& scores)
{
  scores.clear();
  scores.reserve(points.size());
  for (auto & point : points)
  {
    scores.push_back(likelihood(point));
  }
}

double NDT::likelihood(const Point& point)
{
  int index = getIndex(point.x, point.y);
  if (index >= 0)
  {
    return cells_[index].score(point);
  }
  return 0.0;
}

int NDT::getIndex(double x, double y)
{
  if (x < origin_x_ || y < origin_y_)
  {
    return -1;
  }

  unsigned int grid_x = (x - origin_x_) / cell_size_;
  unsigned int grid_y = (y - origin_y_) / cell_size_;
  if (grid_x >= size_x_ || grid_y >= size_y_)
  {
    return -1;
  }

  return (grid_y * size_x_) + grid_x;
}

}  // namespace ndt_2d
