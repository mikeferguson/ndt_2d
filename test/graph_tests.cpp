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
#include <ndt_2d/graph.hpp>
#include <rcpputils/filesystem_helper.hpp>

TEST(GraphTests, read_write_test)
{
  const std::string BAG_NAME = "test_graph";
  rcpputils::fs::remove_all(BAG_NAME);

  bool use_barycenter = true;

  {
    ndt_2d::Graph graph(use_barycenter);

    ndt_2d::ScanPtr scan0 = std::make_shared<ndt_2d::Scan>(0);
    std::vector<ndt_2d::Point> points;
    points.resize(3);
    points[0].x = 2.0;
    points[0].y = 3.0;
    points[1].x = 3.0;
    points[1].y = 3.0;
    points[2].x = 4.0;
    points[2].y = 4.0;
    scan0->setPoints(points);
    ndt_2d::Pose2d pose(0.0, 1.0, 0.0);
    scan0->setPose(pose);
    graph.scans.push_back(scan0);

    // Check barycenter calculations
    EXPECT_EQ(3.0, scan0->getBarycenterPose().x);
    EXPECT_EQ((10 / 3.0) + 1, scan0->getBarycenterPose().y);
    EXPECT_EQ(0.0, scan0->getBarycenterPose().theta);

    ndt_2d::ScanPtr scan1 = std::make_shared<ndt_2d::Scan>(1);
    points[0].x = 1.0;
    points[0].y = 1.5;
    points[1].x = 2.0;
    points[1].y = 1.5;
    points[2].x = 3.0;
    points[2].y = 2.5;
    scan1->setPoints(points);
    pose.x = 1.0;
    pose.y = 2.5;
    pose.theta = 0.05;
    scan1->setPose(pose);
    graph.scans.push_back(scan1);

    double c_x = cos(0.05) * 2.0 - sin(0.05) * (5.5 / 3.0) + 1.0;
    double c_y = sin(0.05) * 2.0 + cos(0.05) * (5.5 / 3.0) + 2.5;
    EXPECT_DOUBLE_EQ(c_x, scan1->getBarycenterPose().x);
    EXPECT_DOUBLE_EQ(c_y, scan1->getBarycenterPose().y);
    EXPECT_DOUBLE_EQ(0.05, scan1->getBarycenterPose().theta);

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

    ndt_2d::ScanPtr scan2 = std::make_shared<ndt_2d::Scan>(2);
    points[0].x = 1.0;
    points[0].y = 1.5;
    points[1].x = 2.0;
    points[1].y = 1.5;
    points[2].x = 3.0;
    points[2].y = 2.5;
    scan2->setPoints(points);
    pose.x = 1.0;
    pose.y = 2.3;
    pose.theta = 0.05;
    scan2->setPose(pose);

    c_y = sin(0.05) * 2.0 + cos(0.05) * (5.5 / 3.0) + 2.3;
    EXPECT_DOUBLE_EQ(c_x, scan2->getBarycenterPose().x);
    EXPECT_DOUBLE_EQ(c_y, scan2->getBarycenterPose().y);
    EXPECT_DOUBLE_EQ(0.05, scan2->getBarycenterPose().theta);

    std::vector<size_t> near = graph.findNearest(scan2);
    EXPECT_EQ(2, near.size());
    EXPECT_EQ(0, near[0]);
    EXPECT_EQ(1, near[1]);
  }

  use_barycenter = true;
  ndt_2d::Graph new_graph(use_barycenter, BAG_NAME);
  EXPECT_EQ(2, new_graph.scans.size());
  EXPECT_EQ(3, new_graph.scans[0]->getPoints().size());
  EXPECT_EQ(3, new_graph.scans[1]->getPoints().size());
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
