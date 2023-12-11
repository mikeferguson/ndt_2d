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

#ifndef NDT_2D__CONVERSIONS_HPP_
#define NDT_2D__CONVERSIONS_HPP_

#include <tf2/utils.h>
#include <ndt_2d/ndt_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ndt_2d
{

inline Pose2d fromMsg(const geometry_msgs::msg::Pose & msg)
{
  return Pose2d(msg.position.x,
                msg.position.y,
                tf2::getYaw(msg.orientation));
}

inline Pose2d fromMsg(const geometry_msgs::msg::PoseStamped & msg)
{
  return fromMsg(msg.pose);
}

inline Pose2d fromMsg(const geometry_msgs::msg::Transform & msg)
{
  return Pose2d(msg.translation.x,
                msg.translation.y,
                tf2::getYaw(msg.rotation));
}

inline Pose2d fromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  return fromMsg(msg.transform);
}

inline Eigen::Isometry3d toEigen(const Pose2d & p)
{
  return Eigen::Isometry3d(Eigen::Translation3d(p.x, p.y, 0.0) *
                           Eigen::AngleAxisd(p.theta, Eigen::Vector3d::UnitZ()));
}

}  // namespace ndt_2d

#endif  // NDT_2D__CONVERSIONS_HPP_
