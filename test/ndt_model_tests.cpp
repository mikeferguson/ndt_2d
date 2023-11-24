/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <ndt_2d/ndt_model.hpp>
#include <gtest/gtest.h>

TEST(NdtModelTests, test_ndt_cell)
{
  ndt_2d::Cell cell;

  // Add points to NDT cell
  ndt_2d::Point p(3.5, 3.5);
  cell.addPoint(p);
  cell.addPoint(p);
  p.x = 3.4;
  p.y = 3.45;
  cell.addPoint(p);
  p.x = 3.6;
  p.y = 3.55;
  cell.addPoint(p);

  // Update NDT
  cell.compute();

  EXPECT_DOUBLE_EQ(3.5, cell.mean(0));
  EXPECT_DOUBLE_EQ(3.5, cell.mean(1));
  EXPECT_NEAR(12.255, cell.covariance(0, 0), 0.0001);
  EXPECT_NEAR(0.0, cell.covariance(0, 1), 0.0001);
  EXPECT_NEAR(12.25125, cell.covariance(1, 1), 0.0001);

  p.x = 3.5;
  p.y = 3.5;
  EXPECT_NEAR(1.0, cell.score(p), 0.001);

  p.x = 3.49;
  p.y = 3.49;
  EXPECT_NEAR(1.0, cell.score(p), 0.001);

  p.x = 3.51;
  p.y = 3.49;
  EXPECT_NEAR(1.0, cell.score(p), 0.001);

  p.x = 3.4;
  p.y = 3.45;
  EXPECT_NEAR(1.0, cell.score(p), 0.001);

  p.x = 3.1;
  p.y = 3.2;
  EXPECT_NEAR(0.9834, cell.score(p), 0.001);

  p.x = 0.0;
  p.y = 0.0;
  EXPECT_NEAR(0.1889, cell.score(p), 0.001);
}

TEST(NdtModelTests, test_ndt)
{
  // Create an NDT with cell size of 1m covering a grid of 10x10 meters
  ndt_2d::NDT ndt(1.0, 10.0, 10.0, -5.0, -5.0);

  // Create a scan to input
  ndt_2d::ScanPtr scan(new ndt_2d::Scan());
  scan->pose.x = 0.0;
  scan->pose.y = 0.0;
  scan->pose.theta = 0.0;
  ndt_2d::Point p(3.5, 3.5);
  scan->points.push_back(p);
  p.x = 3.45;
  p.y = 3.4;
  scan->points.push_back(p);
  p.x = 3.55;
  p.y = 3.6;
  scan->points.push_back(p);

  // Update NDT
  ndt.addScan(scan);
  ndt.compute();

  // Build vector of points to score
  std::vector<ndt_2d::Point> points;
  p.x = 3.5;
  p.y = 3.5;
  points.push_back(p);

  // Test scoring
  double score = ndt.likelihood(points);
  EXPECT_EQ(1.0, score);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
