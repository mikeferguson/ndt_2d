/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#ifndef NDT_2D__CONVERSIONS_HPP_
#define NDT_2D__CONVERSIONS_HPP_

#include <tf2/utils.h>
#include <ndt_2d/ndt_model.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ndt_2d
{

// These conversions are in a separate file to prevent ROS dependencies in core lib

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

Pose2d fromMsg(const geometry_msgs::msg::Transform & msg)
{
  return Pose2d(msg.translation.x,
                msg.translation.y,
                tf2::getYaw(msg.rotation));
}

Pose2d fromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  return fromMsg(msg.transform);
}

}  // namespace ndt_2d

#endif  // NDT_2D__CONVERSIONS_HPP_
