/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__PARTICLE_FILTER_HPP_
#define NDT_2D__PARTICLE_FILTER_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <ndt_2d/motion_model.hpp>
#include <ndt_2d/ndt_model.hpp>

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
   * @param ndt The NDT model to compare localized scans against.
   * @param scan The current scan data.
   */
  void measure(const std::shared_ptr<NDT> & ndt, const ScanPtr & scan);

  /**
   * @brief Resample particles, according to the current weights.
   */
  void resample();

  /** @brief Get the mean of the particle distribution. */
  Eigen::Vector3d getMean();

  /** @brief Get the covariance matrix for the particle distribution. */
  Eigen::Matrix3d getCovariance();

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
};

}  // namespace ndt_2d

#endif  // NDT_2D__PARTICLE_FILTER_HPP_
