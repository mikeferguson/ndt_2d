/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>
#include <ndt_2d/ceres_solver.hpp>
#include <ndt_2d/ceres_solver_ndt.hpp>
#include <ndt_2d/ceres_solver_pose.hpp>

namespace ndt_2d
{
CeresSolver::CeresSolver()
: num_odom_constraints_(0),
  num_loop_constraints_(0),
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

bool CeresSolver::matchScan(const std::shared_ptr<NDT> & ndt,
                            const ScanPtr & scan,
                            Pose2d & pose)
{
  ceres::Problem problem;
  double estimate[3] = {scan->pose.x, scan->pose.y, scan->pose.theta};

  ceres::Solver::Options options;
  options.max_num_iterations = 100;
  options.linear_solver_type = ceres::DENSE_QR;
  options.use_nonmonotonic_steps = true;
  options.function_tolerance = 1e-8;

  ceres::CostFunction * ndt_cost_function =
    CostFunctionNDT::Create(ndt, scan);

  problem.AddResidualBlock(ndt_cost_function,
                           NULL,  // squared loss
                           estimate);

  if (verbose_)
  {
    double ** params = new double*[1];
    params[0] = estimate;
    double * residual = new double[scan->points.size()];
    ndt_cost_function->Evaluate(params, residual, NULL);
    for (size_t i = 0; i < scan->points.size(); ++i)
    {
      std::cout << residual[i] << ", ";
    }
    std::cout << std::endl;
    delete[] residual;
  }

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  if (true || !summary.IsSolutionUsable())
  {
    std::cout << summary.FullReport() << std::endl;
  }

  if (!summary.IsSolutionUsable())
  {
    return false;
  }

  pose.x = estimate[0] - scan->pose.x;
  pose.y = estimate[1] - scan->pose.y;
  pose.theta = estimate[2] - scan->pose.theta;
  return true;
}

bool CeresSolver::optimize(const std::vector<ConstraintPtr> & odom_constraints,
                           const std::vector<ConstraintPtr> & loop_constraints,
                           std::vector<ScanPtr> & scans)
{
  if (scans.empty())
  {
    return false;
  }

  // Update node poses
  for (auto & scan : scans)
  {
    auto node = nodes_.find(scan->id);
    if (node != nodes_.end())
    {
      // Update the pose
      node->second(0) = scan->pose.x;
      node->second(1) = scan->pose.y;
      node->second(2) = scan->pose.theta;
    }
    else
    {
      // New node, insert it
      Eigen::Vector3d pose;
      pose(0) = scan->pose.x;
      pose(1) = scan->pose.y;
      pose(2) = scan->pose.theta;
      nodes_.insert(std::pair<int, Eigen::Vector3d>(scan->id, pose));
    }
  }

  // Add new odom constraints to the problem
  size_t num_constraints = odom_constraints.size();
  for (size_t i = num_odom_constraints_; i < num_constraints; ++i)
  {
    addConstraint(odom_constraints[i], scans);
  }
  num_odom_constraints_ = num_constraints;

  // Add new loop constraints to the problem
  num_constraints = loop_constraints.size();
  for (size_t i = num_loop_constraints_; i < num_constraints; ++i)
  {
    addConstraint(loop_constraints[i], scans);
  }
  num_loop_constraints_ = num_constraints;

  // Make first node fixed
  auto first_node_ = nodes_.find(scans[0]->id);
  problem_->SetParameterBlockConstant(&first_node_->second(0));
  problem_->SetParameterBlockConstant(&first_node_->second(1));
  problem_->SetParameterBlockConstant(&first_node_->second(2));

  // Run the solver
  ceres::Solver::Summary summary;
  ceres::Solve(options_, problem_, &summary);

  if (verbose_ || !summary.IsSolutionUsable())
  {
    std::cout << summary.FullReport() << std::endl;
  }

  if (!summary.IsSolutionUsable())
  {
    return false;
  }

  // Update corrected poses
  for (auto & scan : scans)
  {
    auto node = nodes_.find(scan->id);
    if (node != nodes_.end())
    {
      // Update the pose
      scan->pose.x = node->second(0);
      scan->pose.y = node->second(1);
      scan->pose.theta = node->second(2);
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
  auto begin_node = nodes_.find(begin->id);
  auto end_node = nodes_.find(end->id);

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
    std::cout << "Residuals for constraint between " << begin->id
              << " and " << end->id << std::endl;
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
