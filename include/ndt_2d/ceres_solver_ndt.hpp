/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__CERES_SOLVER_NDT_HPP_
#define NDT_2D__CERES_SOLVER_NDT_HPP_

#include <Eigen/Core>
#include <ceres/ceres.h>
#include <algorithm>
#include <cmath>
#include <memory>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

class CostFunctionNDT
{
public:
  CostFunctionNDT(const std::shared_ptr<NDT> & ndt, const ScanPtr & scan)
  : ndt_(ndt),
    scan_(scan)
  {
  }

  bool operator()(double const * pose, double * residual) const
  {
    Eigen::Isometry3d transform(Eigen::Translation3d(pose[0], pose[1], 0.0) *
                                Eigen::AngleAxisd(pose[2], Eigen::Vector3d::UnitZ()));

    for (size_t i = 0; i < scan_->points.size(); ++i)
    {
      Eigen::Vector3d pt(scan_->points[i].x, scan_->points[i].y, 1.0);
      pt = transform * pt;
      Point p(pt(0), pt(1));
      residual[i] = 1.0 - ndt_->likelihood(p);
    }

    return true;
  }

  static ceres::CostFunction* Create(const std::shared_ptr<NDT> & ndt,
                                     const ScanPtr & scan)
  {
    return (new ceres::NumericDiffCostFunction<CostFunctionNDT, ceres::CENTRAL, ceres::DYNAMIC, 3>(
              new CostFunctionNDT(ndt, scan),
              ceres::TAKE_OWNERSHIP,
              scan->points.size()));
  }

private:
  const std::shared_ptr<NDT> ndt_;
  const ScanPtr scan_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__CERES_SOLVER_NDT_HPP_
