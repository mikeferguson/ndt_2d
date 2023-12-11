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

#ifndef NDT_2D__MOTION_MODEL_HPP_
#define NDT_2D__MOTION_MODEL_HPP_

#include <Eigen/Core>
#include <memory>
#include <random>
#include <vector>
#include <ndt_2d/ndt_model.hpp>

namespace ndt_2d
{

class MotionModel
{
public:
  /**
   * @brief Motion model, based on chapter 5 of Probabilistic Robotics.
   * @param 
   */
  MotionModel(double a1, double a2, double a3, double a4, double a5);

  /**
   * @brief Sample from the motion model.
   * @param dx Change in x coordinate, in robot centric frame.
   * @param dy Change in y coordinate, in robot centric frame.
   * @param dth Change in orientation, in robot centric frame.
   * @param poses The poses to update.
   */
  void sample(const double dx, const double dy, const double dth,
              std::vector<Eigen::Vector3d> & poses);

private:
  double a1_, a2_, a3_, a4_, a5_;

  std::random_device random_;
  std::mt19937 gen_;
};

using MotionModelPtr = std::shared_ptr<MotionModel>;

}  // namespace ndt_2d

#endif  // NDT_2D__MOTION_MODEL_HPP_
