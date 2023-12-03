/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <ndt_2d/ceres_solver.hpp>
#include <gtest/gtest.h>

TEST(CeresSolverTests, information_matrix_tests)
{
  Eigen::Matrix3d covariance;
  covariance(0, 0) = 0.001;
  covariance(1, 1) = 0.001;
  covariance(2, 2) = 0.005;

  Eigen::Matrix3d info = covariance.inverse();

  EXPECT_DOUBLE_EQ(1000, info(0, 0));
  EXPECT_DOUBLE_EQ(1000, info(1, 1));
  EXPECT_DOUBLE_EQ(200,  info(2, 2));
}

TEST(CeresSolverTests, test_solver)
{
  ndt_2d::CeresSolver solver;

  std::vector<ndt_2d::ScanPtr> scans;
  std::vector<ndt_2d::ConstraintPtr> constraints;

  // Optimizer can't run without scans
  EXPECT_FALSE(solver.optimize(constraints, scans));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
