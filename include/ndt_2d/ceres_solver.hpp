/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__CERES_SOLVER_HPP_
#define NDT_2D__CERES_SOLVER_HPP_

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <unordered_map>
#include <vector>
#include <ndt_2d/graph.hpp>

namespace ndt_2d
{

class CeresSolver
{
public:
  CeresSolver();

  /** @brief Turn on/off very verbose outputs. */
  void setVerbose(bool verbose);

  /**
   * @brief Optimize the graph.
   * @param constraints Constraint edges.
   * @param scans Scans, which include the poses to optimize.
   */
  bool optimize(const std::vector<ConstraintPtr> & constraints,
                std::vector<ScanPtr> & scans);

private:
  ceres::ResidualBlockId addConstraint(const ConstraintPtr & constraint,
                                       std::vector<ScanPtr> & scans);

  // Incrementally build the problem to avoid large initialization time
  ceres::Problem * problem_;
  ceres::Solver::Options options_;

  // Duplicate each node/scan pose so that failed optimizations don't break our map
  // Using an unordered map rather than vector so that pointer to Vector3d are not invalidated
  std::unordered_map<int, Eigen::Vector3d> nodes_;

  // To incrementally build, track number of constraints
  size_t num_constraints_;

  bool verbose_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__CERES_SOLVER_HPP_
