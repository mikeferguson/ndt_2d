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
  constraint->begin = from->getId();
  constraint->end = to->getId();
  // Get the delta in map coordinates
  double dx = to->getPose().x - from->getPose().x;
  double dy = to->getPose().y - from->getPose().y;
  // Convert dx/dy into the coordinate frame of begin->pose
  double costh = cos(from->getPose().theta);
  double sinth = sin(from->getPose().theta);
  constraint->transform(0) = costh * dx + sinth * dy;
  constraint->transform(1) = -sinth * dx + costh * dy;
  constraint->transform(2) = to->getPose().theta - from->getPose().theta;
  // Create information matrix
  constraint->information = covariance.inverse();
  // Default is not switchable
  constraint->switchable = false;
  return constraint;
}

}  // namespace ndt_2d
