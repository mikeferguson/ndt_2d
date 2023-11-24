/*
 * Copyright 2023 Michael Ferguson
 * All Rights Reserved
 */

#include <iostream>
#include <ndt_2d/graph.hpp>
#include <ndt_2d/msg/scan.hpp>
#include <ndt_2d/msg/constraint.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rclcpp/serialization.hpp>

namespace ndt_2d
{

Graph::Graph()
{
}

Graph::~Graph()
{
}

Graph::Graph(const std::string & filename)
{
  rosbag2_cpp::Reader reader;
  reader.open(filename);

  rclcpp::Serialization<ndt_2d::msg::Scan> scan_serialization;
  rclcpp::Serialization<ndt_2d::msg::Constraint> constraint_serialization;

  while (reader.has_next())
  {
    rosbag2_storage::SerializedBagMessageSharedPtr msg = reader.read_next();
    rclcpp::SerializedMessage serialized_msg(*msg->serialized_data);

    if (msg->topic_name == "scans")
    {
      ndt_2d::msg::Scan::SharedPtr scan_msg = std::make_shared<ndt_2d::msg::Scan>();
      scan_serialization.deserialize_message(&serialized_msg, scan_msg.get());

      ScanPtr scan = std::make_shared<Scan>();
      scan->id = scan_msg->id;
      scan->pose.x = scan_msg->pose.position.x;
      scan->pose.y = scan_msg->pose.position.y;
      scan->pose.theta = scan_msg->pose.orientation.w;
      scan->points.resize(scan_msg->points.size());
      for (size_t i = 0; i < scan_msg->points.size(); ++i)
      {
        scan->points[i].x = scan_msg->points[i].x;
        scan->points[i].y = scan_msg->points[i].y;
      }
      scans.push_back(scan);
    }
    else
    {
      ndt_2d::msg::Constraint::SharedPtr constraint_msg =
        std::make_shared<ndt_2d::msg::Constraint>();
      constraint_serialization.deserialize_message(&serialized_msg, constraint_msg.get());

      ConstraintPtr constraint = std::make_shared<Constraint>();
      constraint->begin = constraint_msg->begin;
      constraint->end = constraint_msg->end;
      constraint->transform(0) = constraint_msg->transform.translation.x;
      constraint->transform(0) = constraint_msg->transform.translation.y;
      constraint->transform(1) = constraint_msg->transform.translation.z;
      // TODO(fergs): compute information matrix
      if (msg->topic_name == "odom_constraints")
      {
        odom_constraints.push_back(constraint);
      }
      else if (msg->topic_name == "loop_constraints")
      {
        loop_constraints.push_back(constraint);
      }
    }
  }
}

bool Graph::save(const std::string & filename)
{
  // Open the graph file
  rosbag2_cpp::Writer writer;
  writer.open(filename);

  rclcpp::Serialization<ndt_2d::msg::Scan> scan_serialization;
  rclcpp::Serialization<ndt_2d::msg::Constraint> constraint_serialization;

  rclcpp::Time t;

  for (auto & scan : scans)
  {
    // Convert scan into a ROS message
    ndt_2d::msg::Scan::SharedPtr msg = std::make_shared<ndt_2d::msg::Scan>();
    msg->id = scan->id;
    msg->pose.position.x = scan->pose.x;
    msg->pose.position.y = scan->pose.y;
    msg->pose.orientation.w = scan->pose.theta;
    msg->points.resize(scan->points.size());
    for (size_t i = 0; i < scan->points.size(); ++i)
    {
      msg->points[i].x = scan->points[i].x;
      msg->points[i].y = scan->points[i].y;
    }
    // Save message to bagfile
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
      std::make_shared<rclcpp::SerializedMessage>();
    scan_serialization.serialize_message(msg.get(), serialized_msg.get());
    writer.write(serialized_msg, "scans", "ndt_2d/msg/Scan", t);
  }

  for (auto & constraint : odom_constraints)
  {
    // Convert constraint into ROS message
    ndt_2d::msg::Constraint::SharedPtr msg = std::make_shared<ndt_2d::msg::Constraint>();
    msg->begin = constraint->begin;
    msg->end = constraint->end;
    msg->transform.translation.x = constraint->transform(0);
    msg->transform.translation.y = constraint->transform(1);
    msg->transform.translation.z = constraint->transform(2);
    // Save message to bagfile
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
      std::make_shared<rclcpp::SerializedMessage>();
    constraint_serialization.serialize_message(msg.get(), serialized_msg.get());
    writer.write(serialized_msg, "odom_constraints", "ndt_2d/msg/Constraint", t);
  }

  for (auto & constraint : loop_constraints)
  {
    // Convert constraint into ROS message
    ndt_2d::msg::Constraint::SharedPtr msg = std::make_shared<ndt_2d::msg::Constraint>();
    msg->begin = constraint->begin;
    msg->end = constraint->end;
    msg->transform.translation.x = constraint->transform(0);
    msg->transform.translation.y = constraint->transform(1);
    msg->transform.translation.z = constraint->transform(2);
    // Save message to bagfile
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
      std::make_shared<rclcpp::SerializedMessage>();
    constraint_serialization.serialize_message(msg.get(), serialized_msg.get());
    writer.write(serialized_msg, "loop_constraints", "ndt_2d/msg/Constraint", t);
  }

  return true;
}

}  // namespace ndt_2d
