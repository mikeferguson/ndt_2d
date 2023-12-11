/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__POSE_2D_HPP_
#define NDT_2D__POSE_2D_HPP_

namespace ndt_2d
{

struct Pose2d
{
  Pose2d()
  {
    x = 0.0;
    y = 0.0;
    theta = 0.0;
  }

  Pose2d(double x, double y, double theta)
  {
    this->x = x;
    this->y = y;
    this->theta = theta;
  }

  // Pose position in meters
  double x, y;
  // Pose orientation in radians
  double theta;
};

}  // namespace ndt_2d

#endif  // NDT_2D__POSE_2D_HPP_
