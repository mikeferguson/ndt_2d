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

TEST(ParticleTests, test_kd_tree)
{
  ndt_2d::KDTree<double> tree(0.5, 0.5, 0.25, 1 /* try to force re-alloc*/);
  EXPECT_EQ(0, tree.getLeafCount());

  Eigen::Vector3d pose(0.0, 0.0, 0.0);
  double weight = 1.0;
  tree.insert(pose, weight);
  EXPECT_EQ(1, tree.getLeafCount());

  tree.insert(pose, weight);
  EXPECT_EQ(1, tree.getLeafCount());

  pose(0) = 0.75;
  tree.insert(pose, weight);
  EXPECT_EQ(2, tree.getLeafCount());

  pose(0) = -0.75;
  tree.insert(pose, weight);
  EXPECT_EQ(3, tree.getLeafCount());

  pose(0) = 0.75;
  pose(1) = 0.75;
  tree.insert(pose, weight);
  EXPECT_EQ(4, tree.getLeafCount());
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
  // Now move forward
  model->sample(1.0, 0.0, 0.0, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 1.0, 0.3);
  EXPECT_NEAR(mean(2), 1.57, 0.3);
  // Move forward again
  model->sample(1.0, 0.0, 0.0, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.5);
  EXPECT_NEAR(mean(1), 2.0, 0.5);
  EXPECT_NEAR(mean(2), 1.57, 0.5);

  // Test in place rotation in opposite direction
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(0.0, 0.0, -1.57, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), -1.57, 0.3);
  // Now move forward
  model->sample(1.0, 0.0, 0.0, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), -1.0, 0.3);
  EXPECT_NEAR(mean(2), -1.57, 0.3);
  // Move forward again
  model->sample(1.0, 0.0, 0.0, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.5);
  EXPECT_NEAR(mean(1), -2.0, 0.5);
  EXPECT_NEAR(mean(2), -1.57, 0.5);

  // Test in place rotation will small disturbance
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(0.01, -0.01, 1.57, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), 1.57, 0.3);

  // Test in place rotation in other direction will small disturbance
  poses.assign(50, Eigen::Vector3d(0.0, 0.0, 0.0));
  model->sample(0.01, -0.01, -1.57, poses);
  mean = getMean(poses);
  EXPECT_NEAR(mean(0), 0.0, 0.3);
  EXPECT_NEAR(mean(1), 0.0, 0.3);
  EXPECT_NEAR(mean(2), -1.57, 0.3);

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
  filter.resample(0.99, 0.01);
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
