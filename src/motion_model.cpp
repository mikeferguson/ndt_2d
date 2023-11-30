/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <angles/angles.h>
#include <cmath>
#include <ndt_2d/motion_model.hpp>

namespace ndt_2d
{

#define angle_diff angles::shortest_angular_distance
#define angle_norm angles::normalize_angle

MotionModel::MotionModel(double a1, double a2, double a3, double a4, double a5)
: a1_(a1), a2_(a2), a3_(a3), a4_(a4), a5_(a5),
  gen_(random_())
{
}

void MotionModel::sample(const double dx, const double dy, const double dth,
                         std::vector<Eigen::Vector3d> & poses)
{
  // Decompose relative motion
  double trans = std::hypot(dx, dy);
  double rot1 = (trans > 0.01) ? atan2(dy, dx) : 0.0;
  double rot2 = angle_diff(rot1, dth);

  // Reverse motion should not cause massive errors
  double rot1_ = std::min(std::fabs(angle_diff(rot1, 0.0)),
                          std::fabs(angle_diff(rot1, M_PI)));
  double rot2_ = std::min(std::fabs(angle_diff(rot2, 0.0)),
                          std::fabs(angle_diff(rot2, M_PI)));

  // Determine standard deviation
  double sigma_rot1 = std::sqrt(a1_ * rot1_ * rot1_ +
                                a2_ * trans * trans);
  double sigma_trans = std::sqrt(a3_ * trans * trans +
                                 a4_ * rot1_ * rot1_ +
                                 a4_ * rot2_ * rot2_);
  double sigma_rot2 = std::sqrt(a1_ * rot2_ * rot2_ +
                                a2_ * trans * trans);

  // Create distributions
  std::normal_distribution<float> sample_rot1(rot1, sigma_rot1);
  std::normal_distribution<float> sample_trans(trans, sigma_trans);
  std::normal_distribution<float> sample_rot2(rot2, sigma_rot2);

  for (auto & pose : poses)
  {
    float r1 = sample_rot1(gen_);
    float t = sample_trans(gen_);
    float r2 = sample_rot2(gen_);

    pose(0) += t * cos(pose(2) + r1);
    pose(1) += t * sin(pose(2) + r1);
    pose(2) = angle_norm(pose(2) + r1 + r2);
  }
}

}  // namespace ndt_2d
