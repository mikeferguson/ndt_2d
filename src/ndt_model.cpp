/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>

#include <cmath>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

void Cell::addPoint(Point & point)
{
  points.push_back(point);
  valid = false;
}

void Cell::compute()
{
  // No need to update
  if (valid)
  {
    return;
  }

  size_t size = points.size();

  // Compute the mean of the points
  double sum_x = 0.0;
  double sum_y = 0.0;
  for (auto point : points)
  {
    sum_x += point.x;
    sum_y += point.y;
  }
  mean_x = sum_x / size;
  mean_y = sum_y / size;
  // std::cout << "Mean: " << mean_x << " " << mean_y << std::endl;

  // Compute the covariance of the points
  double sum_xx = 0.0;
  double sum_xy = 0.0;
  double sum_yy = 0.0;
  for (auto point : points)
  {
    double dx = point.x - mean_x;
    double dy = point.y - mean_y;
    sum_xx += dx * dx;
    sum_xy += dx * dy;
    sum_yy += dy * dy;
  }
  cov_xx = sum_xx / size;
  cov_xy = sum_xy / size;
  cov_yy = sum_yy / size;
  // std::cout << "Cov: " << cov_xx << " " << cov_xy << " " << cov_yy << std::endl;

  // Compute the inverse of the covariance
  double determinant = (cov_xx * cov_yy) - (cov_xy * cov_xy);
  if (determinant > 0.0 && determinant < 0.000001) determinant = 0.000001;
  if (determinant < 0.0 && determinant > -0.000001) determinant = -0.000001;
  inv_xx = cov_yy / determinant;
  inv_xy = -cov_xy / determinant;
  inv_yy = cov_xx / determinant;
  // std::cout << "Det: " << determinant << std::endl;
  // std::cout << "Inv: " << inv_xx << " " << inv_xy << " " << cov_yy << std::endl;

  valid = true;
}

double Cell::score(Point & p)
{
  if (points.size() < 2)
  {
    // Need at least two points for our mean/cov to be valid
    return 0.0;
  }

  // std::cout << "Model: " << mean_x << " " << mean_y << " ";
  // std::cout << cov_xx << " " << cov_xy << " " << cov_yy << std::endl;

  double dx = p.x - mean_x;
  double dy = p.y - mean_y;

  double r1c1 = (dx * inv_xx) + (dy * inv_xy);
  double r1c2 = (dx * inv_xy) + (dy * inv_yy);

  double score = exp(r1c1 * dx + r1c2 * dy);
  // std::cout << "scoring: " << dx << ", " << dy << ": " << r1c1;
  // std::cout << ", " << r1c2 << " " << score << std::endl;

  return score;
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

void NDT::addScan(ScanPtr& scan, Pose2d& pose)
{
  // Precompute transforms
  double cos_th = cos(pose.theta);
  double sin_th = sin(pose.theta);

  for (auto & point : scan->points)
  {
    // Transform the point by pose
    Point p(pose.x, pose.y);
    p.x += point.x * cos_th + point.y * sin_th;
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

double NDT::likelihood(std::vector<Point>& points)
{
  double score = 0.0;
  for (auto & point : points)
  {
    score += likelihood(point);
  }
  return score;
}

void NDT::likelihood(std::vector<Point>& points, std::vector<double>& scores)
{
  scores.clear();
  scores.reserve(points.size());
  for (auto & point : points)
  {
    scores.push_back(likelihood(point));
  }
}

double NDT::likelihood(Point& point)
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
