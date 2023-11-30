/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <angles/angles.h>
#include <random>
#include <ndt_2d/particle_filter.hpp>

namespace ndt_2d
{

ParticleFilter::ParticleFilter(size_t min_particles, size_t max_particles,
    MotionModelPtr & motion_model)
: gen_(random_()),
  motion_model_(motion_model),
  min_particles_(min_particles),
  max_particles_(max_particles),
  mean_(Eigen::Vector3d::Zero()),
  cov_(Eigen::Matrix3d::Zero())
{
  particles_.reserve(max_particles_);
  weights_.reserve(max_particles_);
  particles_.assign(min_particles_, Particle(0.0, 0.0, 0.0));
  weights_.assign(min_particles_, 1.0 / min_particles_);
  updateStatistics();
}

void ParticleFilter::init(const double x, const double y, const double theta,
                          const double sigma_x, const double sigma_y, const double sigma_theta)
{
  std::normal_distribution<float> sample_x(x, sigma_x);
  std::normal_distribution<float> sample_y(y, sigma_y);
  std::normal_distribution<float> sample_theta(theta, sigma_theta);

  for (auto & particle : particles_)
  {
    particle(0) = sample_x(gen_);
    particle(1) = sample_y(gen_);
    particle(2) = angles::normalize_angle(sample_theta(gen_));
  }

  weights_.assign(particles_.size(), 1.0 / particles_.size());
  updateStatistics();
}

void ParticleFilter::update(const double dx, const double dy, const double dth)
{
  // Apply control update
  motion_model_->sample(dx, dy, dth, particles_);
  updateStatistics();
}

void ParticleFilter::measure(const std::shared_ptr<NDT> & ndt, const ScanPtr & scan)
{
  for (size_t i = 0; i < particles_.size(); ++i)
  {
    // Pose of this particle in NDT format
    Pose2d pose(particles_[i](0), particles_[i](1), particles_[i](2));
    // Compute the score, ignoring the scan->pose
    weights_[i] = ndt->likelihood(scan->points, pose);
  }
  updateStatistics();
}

void ParticleFilter::resample()
{
  std::discrete_distribution<size_t> d(weights_.begin(), weights_.end());

  std::vector<Particle> resampled;
  resampled.reserve(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i)
  {
    resampled.push_back(particles_[d(gen_)]);
  }

  particles_ = resampled;

  updateStatistics();
}

Eigen::Vector3d ParticleFilter::getMean()
{
  return mean_;
}

Eigen::Matrix3d ParticleFilter::getCovariance()
{
  return cov_;
}

void ParticleFilter::getMsg(geometry_msgs::msg::PoseArray & msg)
{
  msg.poses.reserve(particles_.size());
  for (auto & particle : particles_)
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = particle(0);
    pose.position.y = particle(1);
    pose.orientation.z = sin(particle(2) / 2.0);
    pose.orientation.w = cos(particle(2) / 2.0);
    msg.poses.push_back(pose);
  }
}

void ParticleFilter::updateStatistics()
{
  // Recompute weights to sum to 1.0
  double sum_weight = 0.0;
  for (auto & w : weights_)
  {
    sum_weight += w;
  }
  for (auto & w : weights_)
  {
    w /= sum_weight;
  }

  // Temporary mean and correlation
  Eigen::Vector3d mean = Eigen::Vector3d::Zero();
  Eigen::Matrix3d corr = Eigen::Matrix3d::Zero();
  // Use circular mean for theta
  double sum_cos_th = 0.0, sum_sin_th = 0.0;

  for (size_t i = 0; i < particles_.size(); ++i)
  {
    mean += weights_[i] * particles_[i];
    sum_cos_th += weights_[i] * cos(particles_[i](2));
    sum_sin_th += weights_[i] * sin(particles_[i](2));

    for (size_t j = 0; j < 2; ++j)
    {
      for (size_t k = j; k < 2; ++k)
      {
        corr(j, k) += weights_[i] * particles_[i](j) * particles_[i](k);
      }
    }
  }

  // Mean is already normalized, since weights were normalized
  mean_(0) = mean(0);
  mean_(1) = mean(1);
  mean_(2) = atan2(sum_sin_th, sum_cos_th);

  // Compute covariance for x/y
  for (size_t j = 0; j < 2; ++j)
  {
    for (size_t k = j; k < 2; ++k)
    {
      cov_(j, k) = corr(j, k) - mean(j) * mean(k);
      cov_(k, j) = cov_(j, k);
    }
  }

  // Compute covariance for theta
  for (size_t i = 0; i < particles_.size(); ++i)
  {
    double d = angles::shortest_angular_distance(particles_[i](2), mean_(2));
    cov_(2, 2) += weights_[i] * d * d;
  }
}

}  // namespace ndt_2d
