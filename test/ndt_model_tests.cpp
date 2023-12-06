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
  Eigen::Vector2d p(3.5, 3.5);
  cell.addPoint(p);
  cell.addPoint(p);
  p(0) = 3.4;
  p(1) = 3.45;
  cell.addPoint(p);
  p(0) = 3.6;
  p(1) = 3.55;
  cell.addPoint(p);

  // Update NDT
  EXPECT_FALSE(cell.valid);
  cell.compute();
  EXPECT_TRUE(cell.valid);

  // Mean should be correct
  EXPECT_DOUBLE_EQ(3.5, cell.mean(0));
  EXPECT_DOUBLE_EQ(3.5, cell.mean(1));

  // But score will be 0 since we don't have enough points
  p(0) = 3.5;
  p(1) = 3.5;
  EXPECT_NEAR(0.0, cell.score(p), 0.001);

  // Add a few more points
  p(0) = 3.6;
  p(1) = 3.45;
  cell.addPoint(p);
  p(0) = 3.4;
  p(1) = 3.55;
  cell.addPoint(p);

  // Update NDT
  cell.compute();

  EXPECT_NEAR(0.008, cell.covariance(0, 0), 0.001);
  EXPECT_NEAR(0.0, cell.covariance(0, 1), 0.001);
  EXPECT_NEAR(0.002, cell.covariance(1, 1), 0.001);

  // Test at mean
  p(0) = 3.5;
  p(1) = 3.5;
  EXPECT_NEAR(1.0, cell.score(p), 0.001);

  // One std dev away in x
  p(0) = 3.5 + sqrt(0.008);
  p(1) = 3.5;
  EXPECT_NEAR(0.6065, cell.score(p), 0.001);

  // Two std dev away in x
  p(0) = 3.5 + 2 * sqrt(0.008);
  p(1) = 3.5;
  EXPECT_NEAR(0.1353, cell.score(p), 0.001);

  // One std dev away in y
  p(0) = 3.5;
  p(1) = 3.5 + sqrt(0.002);
  EXPECT_NEAR(0.6065, cell.score(p), 0.001);

  // Two std dev away in y
  p(0) = 3.5;
  p(1) = 3.5 + 2 * sqrt(0.002);
  EXPECT_NEAR(0.1353, cell.score(p), 0.001);

  // Really far away
  p(0) = 0.0;
  p(1) = 0.0;
  EXPECT_NEAR(0.0, cell.score(p), 0.001);
}

TEST(NdtModelTests, test_ndt_cell_no_x_variation)
{
  ndt_2d::Cell cell;

  // Add points to NDT cell
  Eigen::Vector2d p(3.5, 3.5);
  cell.addPoint(p);
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.correlation(1, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(1, 1));

  p(1) = 3.45;
  cell.addPoint(p);
  cell.addPoint(p);

  p(1) = 3.55;
  cell.addPoint(p);
  cell.addPoint(p);
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.correlation(1, 0));
  EXPECT_DOUBLE_EQ(12.252, cell.correlation(1, 1));

  // Update NDT
  cell.compute();

  // Mean should be correct
  EXPECT_DOUBLE_EQ(3.5, cell.mean(0));
  EXPECT_DOUBLE_EQ(3.5, cell.mean(1));

  // Check internal matrice are correct
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(0, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(1, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(0, 1));
  EXPECT_NEAR(0.0025, cell.covariance(1, 1), 0.000001);
  EXPECT_NEAR(400000.0, cell.information(0, 0), 0.000001);
  EXPECT_DOUBLE_EQ(0.0, cell.information(1, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.information(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.information(1, 1));
}

TEST(NdtModelTests, test_ndt_cell_no_y_variation)
{
  ndt_2d::Cell cell;

  // Add points to NDT cell
  Eigen::Vector2d p(3.5, 3.5);
  cell.addPoint(p);
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.correlation(1, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(1, 1));

  p(0) = 3.45;
  cell.addPoint(p);
  cell.addPoint(p);

  p(0) = 3.55;
  cell.addPoint(p);
  cell.addPoint(p);
  EXPECT_DOUBLE_EQ(12.252, cell.correlation(0, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.correlation(1, 0));
  EXPECT_DOUBLE_EQ(12.25, cell.correlation(1, 1));

  // Update NDT
  cell.compute();

  // Mean should be correct
  EXPECT_DOUBLE_EQ(3.5, cell.mean(0));
  EXPECT_DOUBLE_EQ(3.5, cell.mean(1));

  // Check internal matrice are correct
  EXPECT_NEAR(0.0025, cell.covariance(0, 0), 0.000001);
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(1, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(0, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.covariance(1, 1));
  EXPECT_DOUBLE_EQ(0.0, cell.information(0, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.information(1, 0));
  EXPECT_DOUBLE_EQ(0.0, cell.information(0, 1));
  EXPECT_NEAR(400000.0, cell.information(1, 1), 0.000001);
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
  p.x = 3.45;
  p.y = 3.6;
  scan->points.push_back(p);
  p.x = 3.45;
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
  EXPECT_NEAR(0.7659, score, 0.001);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
