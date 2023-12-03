/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <gtest/gtest.h>
#include <ndt_2d/graph.hpp>
#include <rcpputils/filesystem_helper.hpp>

TEST(GraphTests, read_write_test)
{
  const std::string BAG_NAME = "test_graph";
  rcpputils::fs::remove_all(BAG_NAME);

  {
    ndt_2d::Graph graph;

    ndt_2d::ScanPtr scan0 = std::make_shared<ndt_2d::Scan>();
    scan0->id = 0;
    scan0->points.resize(3);
    scan0->points[0].x = 2.0;
    scan0->points[0].y = 3.0;
    scan0->points[1].x = 3.0;
    scan0->points[1].y = 3.0;
    scan0->points[2].x = 4.0;
    scan0->points[2].y = 4.0;
    scan0->pose.x = 0.0;
    scan0->pose.y = 1.0;
    scan0->pose.theta = 0.0;
    graph.scans.push_back(scan0);

    ndt_2d::ScanPtr scan1 = std::make_shared<ndt_2d::Scan>();
    scan1->id = 1;
    scan1->points.resize(3);
    scan1->points[0].x = 1.0;
    scan1->points[0].y = 1.5;
    scan1->points[1].x = 2.0;
    scan1->points[1].y = 1.5;
    scan1->points[2].x = 3.0;
    scan1->points[2].y = 2.5;
    scan1->pose.x = 1.0;
    scan1->pose.y = 2.5;
    scan1->pose.theta = 0.05;
    graph.scans.push_back(scan1);

    ndt_2d::ConstraintPtr constraint = std::make_shared<ndt_2d::Constraint>();
    constraint->begin = 0;
    constraint->end = 1;
    constraint->transform(0) = 1.0;
    constraint->transform(1) = 1.5;
    constraint->transform(2) = 0.0;
    constraint->information = Eigen::Matrix3d::Zero();
    constraint->information(0, 0) = 100.0;
    constraint->information(1, 1) = 100.0;
    constraint->information(2, 2) = 20.0;
    constraint->switchable = true;
    graph.constraints.push_back(constraint);

    // Save simple graph for later testing of the load function
    graph.save(BAG_NAME);

    ndt_2d::ScanPtr scan2 = std::make_shared<ndt_2d::Scan>();
    scan2->id = 2;
    scan2->points.resize(3);
    scan2->points[0].x = 1.0;
    scan2->points[0].y = 1.5;
    scan2->points[1].x = 2.0;
    scan2->points[1].y = 1.5;
    scan2->points[2].x = 3.0;
    scan2->points[2].y = 2.5;
    scan2->pose.x = 1.0;
    scan2->pose.y = 2.3;
    scan2->pose.theta = 0.05;

    std::vector<size_t> near = graph.findNearest(scan2);
    EXPECT_EQ(2, near.size());
    EXPECT_EQ(1, near[0]);
    EXPECT_EQ(0, near[1]);
  }

  ndt_2d::Graph new_graph(BAG_NAME);
  EXPECT_EQ(2, new_graph.scans.size());
  EXPECT_EQ(3, new_graph.scans[0]->points.size());
  EXPECT_EQ(3, new_graph.scans[1]->points.size());
  EXPECT_EQ(1, new_graph.constraints.size());

  // Verify constraint save/load
  EXPECT_EQ(0, new_graph.constraints[0]->begin);
  EXPECT_EQ(1, new_graph.constraints[0]->end);
  EXPECT_EQ(1.0, new_graph.constraints[0]->transform(0));
  EXPECT_EQ(1.5, new_graph.constraints[0]->transform(1));
  EXPECT_EQ(0.0, new_graph.constraints[0]->transform(2));
  EXPECT_EQ(100.0, new_graph.constraints[0]->information(0, 0));
  EXPECT_EQ(100.0, new_graph.constraints[0]->information(1, 1));
  EXPECT_EQ(20.0, new_graph.constraints[0]->information(2, 2));
  EXPECT_EQ(true, new_graph.constraints[0]->switchable);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
