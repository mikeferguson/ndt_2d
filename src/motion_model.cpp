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
