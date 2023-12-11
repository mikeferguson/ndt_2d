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

#include <cmath>
#include <ndt_2d/scan.hpp>

namespace ndt_2d
{

Scan::Scan(size_t id) : id_(id), dirty_(true)
{
}

size_t Scan::getId()
{
  return id_;
}

void Scan::setPose(const Pose2d & pose)
{
  dirty_ = true;
  pose_ = pose;
}

Pose2d Scan::getPose()
{
  return pose_;
}

Pose2d Scan::getBarycenterPose()
{
  if (dirty_) update();
  return barycenter_;
}

void Scan::setPoints(const std::vector<Point> & points)
{
  dirty_ = true;
  points_ = points;
}

std::vector<Point> Scan::getPoints()
{
  return points_;
}

void Scan::update()
{
  double cos_theta = cos(pose_.theta);
  double sin_theta = sin(pose_.theta);

  // Compute the barycenter
  barycenter_ = pose_;
  if (!points_.empty())
  {
    Point center;
    for (auto & point : points_)
    {
      center.x += cos_theta * point.x - sin_theta * point.y;
      center.y += sin_theta * point.x + cos_theta * point.y;
    }
    barycenter_.x += center.x / points_.size();
    barycenter_.y += center.y / points_.size();
  }
  dirty_ = false;
}

}  // namespace ndt_2d
