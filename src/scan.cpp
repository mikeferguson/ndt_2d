/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <cmath>
#include <ndt_2d/scan.hpp>

namespace ndt_2d
{

Scan::Scan(size_t id) : id_(id), dirty_(true)
{
}

size_t Scan::getId()
{
  return id_;
}

void Scan::setPose(const Pose2d & pose)
{
  dirty_ = true;
  pose_ = pose;
}

Pose2d Scan::getPose()
{
  return pose_;
}

Pose2d Scan::getBarycenterPose()
{
  if (dirty_) update();
  return barycenter_;
}

void Scan::setPoints(const std::vector<Point> & points)
{
  dirty_ = true;
  points_ = points;
}

std::vector<Point> Scan::getPoints()
{
  return points_;
}

void Scan::update()
{
  double cos_theta = cos(pose_.theta);
  double sin_theta = sin(pose_.theta);

  // Compute the barycenter
  barycenter_ = pose_;
  if (!points_.empty())
  {
    Point center;
    for (auto & point : points_)
    {
      center.x += cos_theta * point.x - sin_theta * point.y;
      center.y += sin_theta * point.x + cos_theta * point.y;
    }
    barycenter_.x += center.x / points_.size();
    barycenter_.y += center.y / points_.size();
  }
  dirty_ = false;
}

}  // namespace ndt_2d
