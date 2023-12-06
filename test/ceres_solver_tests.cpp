/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <ndt_2d/ceres_solver.hpp>
#include <gtest/gtest.h>

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
