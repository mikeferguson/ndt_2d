/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__CONSTRAINT_HPP_
#define NDT_2D__CONSTRAINT_HPP_

#include <Eigen/Core>
#include <memory>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

struct Constraint
{
  size_t begin;
  size_t end;
  // dx, dy, dtheta between begin and end poses
  Eigen::Vector3d transform;
  Eigen::Matrix3d information;
  // Can this constraint be disabled?
  bool switchable;
};

typedef std::shared_ptr<Constraint> ConstraintPtr;

/**
 * @brief Create a constraint between two scans based on their current poses.
 */
ConstraintPtr makeConstraint(const ScanPtr & from,
                             const ScanPtr & to,
                             Eigen::Matrix3d & covariance);

}  // namespace ndt_2d

#endif  // NDT_2D__CONSTRAINT_HPP_
