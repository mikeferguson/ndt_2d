/*
 * Copyright (c) 2023 Michael Ferguson
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the opyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
