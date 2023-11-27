/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
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
