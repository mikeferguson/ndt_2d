/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__POINT_HPP_
#define NDT_2D__POINT_HPP_

namespace ndt_2d
{

struct Point
{
  Point()
  {
    x = 0;
    y = 0;
  }

  Point(double x, double y)
  {
    this->x = x;
    this->y = y;
  }

  // Point location, in meters
  double x, y;
};

}  // namespace ndt_2d

#endif  // NDT_2D__POINT_HPP_
