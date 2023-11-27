/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <gtest/gtest.h>
#include <angles/angles.h>
#include <Eigen/Core>
#include <vector>
#include <iostream>
#include <ndt_2d/motion_model.hpp>
#include <ndt_2d/particle_filter.hpp>

Eigen::Vector3d getMean(std::vector<Eigen::Vector3d>& poses)
{
  Eigen::Vector3d mean(0.0, 0.0, 0.0);
  for (auto & pose : poses)
  {
    mean += pose;
  }
  return (mean / poses.size());
}

TEST(ParticleTests, test_particle_filter)
{
  ndt_2d::MotionModelPtr model =
    std::make_shared<ndt_2d::MotionModel>(0.1, 0.1, 0.1, 0.1, 0.0);

  // Test basic forward motion
  std::vector<Eigen::Vector3d> poses;
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(1.0, 0.0, 0.0, poses);
  Eigen::Vector3d mean = getMean(poses);
  EXPECT_NEAR(mean(0), 1.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), 0.0, 0.3);

  // Test in place rotation
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(0.0, 0.0, 1.57, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), 1.57, 0.3);

  // Test backward motion
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(-1.0, 0.0, 0.0, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), -1.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  // NOTE: need to compute a proper circular mean to check mean(2)

  // Test combined translation + rotation
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(0.3, 0.0, -0.3, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.3, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), -0.3, 0.3);

  // Test the particle filter
  ndt_2d::ParticleFilter filter(50, 100, model);
  filter.init(1.25, 0.5, 1.57, 0.1, 0.1, 0.3);

  // Check the statistics
  mean = filter.getMean();
  EXPECT_NEAR(mean(0), 1.25, 0.2);
  EXPECT_NEAR(mean(1), 0.5, 0.2);
  EXPECT_NEAR(mean(2), 1.57, 0.2);
  Eigen::Matrix3d cov = filter.getCovariance();
  EXPECT_NEAR(cov(0, 0), 0.01, 0.01);
  EXPECT_NEAR(cov(1, 1), 0.01, 0.01);
  EXPECT_NEAR(cov(2, 2), 0.09, 0.1);

  // Move the robot
  filter.update(1.5, 0.0, 0.0);

  // Check the statistics again
  mean = filter.getMean();
  EXPECT_NEAR(mean(0), 1.25, 0.4);
  EXPECT_NEAR(mean(1), 2.0, 0.4);
  EXPECT_NEAR(mean(2), 1.57, 0.4);

  // Test resampling
  filter.resample();
  mean = filter.getMean();
  EXPECT_NEAR(mean(0), 1.25, 0.4);
  EXPECT_NEAR(mean(1), 2.0, 0.4);
  EXPECT_NEAR(mean(2), 1.57, 0.4);

  // Test circular mean for theta
  filter.init(0.0, 0.0, 3.14, 0.1, 0.1, 0.1);
  mean = filter.getMean();
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  double angle = angles::shortest_angular_distance(mean(2), 3.14);
  EXPECT_NEAR(angle, 0.0, 0.3);

  // Test backwards motion / circular mean for theta
  filter.init(0.0, 0.0, 0.0, 0.1, 0.1, 0.1);
  filter.update(-1.0, 0.0, 0.0);
  mean = filter.getMean();
  EXPECT_NEAR(mean(0), -1.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), 0.0, 0.3);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
