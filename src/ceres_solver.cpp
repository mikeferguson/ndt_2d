/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>
#include <ndt_2d/ceres_solver.hpp>
#include <ndt_2d/ceres_solver_pose.hpp>

namespace ndt_2d
{
CeresSolver::CeresSolver()
: num_constraints_(0),
  verbose_(false)
{
  options_.max_num_iterations = 100;
  options_.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;

  problem_ = new ceres::Problem();
}

void CeresSolver::setVerbose(bool verbose)
{
  verbose_ = verbose;
}

bool CeresSolver::optimize(const std::vector<ConstraintPtr> & constraints,
                           std::vector<ScanPtr> & scans)
{
  if (scans.empty())
  {
    return false;
  }

  // Update node poses
  for (auto & scan : scans)
  {
    auto node = nodes_.find(scan->getId());
    if (node != nodes_.end())
    {
      // Update the pose
      node->second(0) = scan->getPose().x;
      node->second(1) = scan->getPose().y;
      node->second(2) = scan->getPose().theta;
    }
    else
    {
      // New node, insert it
      Eigen::Vector3d pose;
      pose(0) = scan->getPose().x;
      pose(1) = scan->getPose().y;
      pose(2) = scan->getPose().theta;
      nodes_.insert(std::pair<int, Eigen::Vector3d>(scan->getId(), pose));
    }
  }

  // Add new constraints to the problem
  size_t n = constraints.size();
  for (size_t i = num_constraints_; i < n; ++i)
  {
    addConstraint(constraints[i], scans);
  }
  num_constraints_ = n;

  // Make first node fixed
  auto first_node_ = nodes_.find(scans[0]->getId());
  problem_->SetParameterBlockConstant(&first_node_->second(0));
  problem_->SetParameterBlockConstant(&first_node_->second(1));
  problem_->SetParameterBlockConstant(&first_node_->second(2));

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);

  if (!summary.IsSolutionUsable())
  {
    return false;
  }

  if (verbose_)
  {
    std::cout << summary.FullReport() << std::endl;
  }

  // Update corrected poses
  for (auto & scan : scans)
  {
    auto node = nodes_.find(scan->getId());
    if (node != nodes_.end())
    {
      // Update the pose
      Pose2d pose(node->second(0), node->second(1), node->second(2));
      scan->setPose(pose);
    }
  }
  return true;
}

ceres::ResidualBlockId CeresSolver::addConstraint(const ConstraintPtr & constraint,
                                                  std::vector<ScanPtr> & scans)
{
  ScanPtr & begin = scans[constraint->begin];
  ScanPtr & end = scans[constraint->end];

  // These are our internal copies of just the node pose
  auto begin_node = nodes_.find(begin->getId());
  auto end_node = nodes_.find(end->getId());

  const Eigen::Matrix3d sqrt_information = constraint->information.llt().matrixL();

  ceres::CostFunction * cost_function =
    PoseGraph2dErrorTerm::Create(constraint->transform(0),
                                 constraint->transform(1),
                                 constraint->transform(2),
                                 sqrt_information);

  if (verbose_)
  {
    double ** param = new double*[6];
    param[0] = &(begin_node->second(0));
    param[1] = &(begin_node->second(1));
    param[2] = &(begin_node->second(2));
    param[3] = &(end_node->second(0));
    param[4] = &(end_node->second(1));
    param[5] = &(end_node->second(2));
    double * residual = new double[3];
    cost_function->Evaluate(param, residual, NULL);
    std::cout << "Residuals for constraint between " << begin->getId()
              << " and " << end->getId() << std::endl;
    std::cout << "  " << residual[0] << ", " << residual[1] << ", "
              << residual[2] << std::endl;
    delete[] param;
    delete[] residual;
  }

  return problem_->AddResidualBlock(cost_function,
                                    NULL,  // squared loss
                                    &(begin_node->second(0)),
                                    &(begin_node->second(1)),
                                    &(begin_node->second(2)),
                                    &(end_node->second(0)),
                                    &(end_node->second(1)),
                                    &(end_node->second(2)));
}

}  // namespace ndt_2d
