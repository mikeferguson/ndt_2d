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

#ifndef NDT_2D__PARTICLE_FILTER_HPP_
#define NDT_2D__PARTICLE_FILTER_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <geometry_msgs/msg/pose_array.hpp>
#include <ndt_2d/kd_tree.hpp>
#include <ndt_2d/motion_model.hpp>
#include <ndt_2d/ndt_model.hpp>
#include <ndt_2d/scan_matcher.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ndt_2d
{

class ParticleFilter
{
  using Particle = Eigen::Vector3d;

public:
  ParticleFilter(size_t min_particles, size_t max_particles,
                 MotionModelPtr & motion_model);

  /**
   * @brief Initialize the particle filter to a pose with given variances.
   * @param x Mean of samples in X direction.
   * @param y Mean of samples in Y direction.
   * @param theta Mean of samples in angular orientation.
   * @param sigma_x Variance of samples in X direction.
   * @param sigma_y Variance of samples in Y direction.
   * @param sigma_theat Variance of samples in angular orientation.
   */
  void init(const double x, const double y, const double theta,
            const double sigma_x, const double sigma_y, const double sigma_theta);

  /**
   * @brief Apply a control update to the particles.
   * @param dx Distance traveled in the X direction, in robot centric frame.
   * @param dy Distance traveled in the Y direction, in robot centric frame.
   * @param dth Change in angular orientation, in robot centric frame.
   */
  void update(const double dx, const double dy, const double dth);

  /**
   * @brief Apply a measurement update.
   * @param matcher Scan localization method.
   * @param scan The current scan data.
   */
  void measure(const ScanMatcherPtr & matcher, const ScanPtr & scan);

  /**
   * @brief Resample particles, according to the current weights.
   * @param kld_err Maximum error between true distribution and estimated distribution.
   * @param kld_z Upper quantile of (1-p), p = probability that error is less than kld_err.
   */
  void resample(const double kld_err, const double kld_z);

  /** @brief Get the mean of the particle distribution. */
  Eigen::Vector3d getMean();

  /** @brief Get the covariance matrix for the particle distribution. */
  Eigen::Matrix3d getCovariance();

  /** @brief Get a visualization message of the particle poses. */
  void getMsg(geometry_msgs::msg::PoseArray & msg);

private:
  // Internal helper: normalize weights, compute mean and covariance
  void updateStatistics();

  std::random_device random_;
  std::mt19937 gen_;

  // Motion model to sample from when applying control updates
  MotionModelPtr motion_model_;

  // Particle information
  size_t min_particles_, max_particles_;
  std::vector<Particle> particles_;
  std::vector<double> weights_;
  Eigen::Vector3d mean_;
  Eigen::Matrix3d cov_;

  // KD Tree for adaptive resampling
  KDTree<double> kd_tree_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__PARTICLE_FILTER_HPP_
