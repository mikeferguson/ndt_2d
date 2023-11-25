/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__CERES_SOLVER_HPP_
#define NDT_2D__CERES_SOLVER_HPP_

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <unordered_map>
#include <memory>
#include <vector>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

class CeresSolver
{
public:
  CeresSolver();

  /** @brief Turn on/off very verbose outputs. */
  void setVerbose(bool verbose);

  /**
   * @brief Match a scan against an NDT map.
   * @param ndt Map to match scan against.
   * @param scan Scan to match against NDT map.
   * @param pose The corrected pose that best matches scan to NDT map.
   * @returns True if scan matcher succeeded.
   */
  bool matchScan(const std::shared_ptr<NDT> & ndt,
                 const ScanPtr & scan,
                 Pose2d & pose);

  /**
   * @brief Optimize a global graph.
   * @param odom_constraints Additional constraint edges.
   * @param 
   */
  bool optimize(const std::vector<ConstraintPtr> & odom_constraints,
                const std::vector<ConstraintPtr> & loop_constraints,
                std::vector<ScanPtr> & scans);

private:
  ceres::ResidualBlockId addConstraint(const ConstraintPtr & constraint,
                                       std::vector<ScanPtr> & scans);

  // Incrementally build the graph problem to avoid large initialization time
  ceres::Problem * problem_;
  ceres::Solver::Options options_;

  // Duplicate each node/scan pose so that failed optimizations don't break our map
  // Using an unordered map rather than vector so that pointer to Vector3d are not invalidated
  std::unordered_map<int, Eigen::Vector3d> nodes_;

  // To incrementally build, track number of constraints
  size_t num_odom_constraints_;
  size_t num_loop_constraints_;

  bool verbose_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__CERES_SOLVER_HPP_
