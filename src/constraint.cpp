/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>
#include <ndt_2d/constraint.hpp>

namespace ndt_2d
{

ConstraintPtr makeConstraint(const ScanPtr & from,
                             const ScanPtr & to,
                             Eigen::Matrix3d & covariance)
{
  ConstraintPtr constraint = std::make_shared<Constraint>();
  constraint->begin = from->id;
  constraint->end = to->id;
  // Get the delta in map coordinates
  double dx = to->pose.x - from->pose.x;
  double dy = to->pose.y - from->pose.y;
  // Convert dx/dy into the coordinate frame of begin->pose
  double costh = cos(from->pose.theta);
  double sinth = sin(from->pose.theta);
  constraint->transform(0) = costh * dx + sinth * dy;
  constraint->transform(1) = -sinth * dx + costh * dy;
  constraint->transform(2) = to->pose.theta - from->pose.theta;
  // Create information matrix
  constraint->information = covariance.inverse();
  // Default is not switchable
  constraint->switchable = false;
  return constraint;
}

}  // namespace ndt_2d
