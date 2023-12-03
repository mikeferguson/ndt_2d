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
      constraint->transform(1) = constraint_msg->transform.translation.y;
      constraint->transform(2) = constraint_msg->transform.translation.z;
      for (size_t i = 0; i < 3; ++i)
      {
        for (size_t j = 0; j < 3; ++j)
        {
          constraint->information(i, j) = constraint_msg->information[i * 3 + j];
        }
      }
      constraint->switchable = constraint_msg->switchable;
      constraints.push_back(constraint);
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

  for (auto & constraint : constraints)
  {
    // Convert constraint into ROS message
    ndt_2d::msg::Constraint::SharedPtr msg = std::make_shared<ndt_2d::msg::Constraint>();
    msg->begin = constraint->begin;
    msg->end = constraint->end;
    msg->transform.translation.x = constraint->transform(0);
    msg->transform.translation.y = constraint->transform(1);
    msg->transform.translation.z = constraint->transform(2);
    for (size_t i = 0; i < 3; ++i)
    {
      for (size_t j = 0; j < 3; ++j)
      {
        msg->information[i * 3 + j] = constraint->information(i, j);
      }
    }
    msg->switchable = constraint->switchable;
    // Save message to bagfile
    std::shared_ptr<rclcpp::SerializedMessage> serialized_msg =
      std::make_shared<rclcpp::SerializedMessage>();
    constraint_serialization.serialize_message(msg.get(), serialized_msg.get());
    writer.write(serialized_msg, "constraints", "ndt_2d/msg/Constraint", t);
  }

  return true;
}

std::vector<size_t> Graph::findNearest(const ScanPtr & scan, double dist, int limit_scan_index)
{
  // Build a KD-tree from scratch each time - no sense with bookkeeping to use dynamic
  // tree, since it rebuilds index from scratch: https://github.com/jlblancoc/nanoflann/issues/90
  size_t limit = (limit_scan_index > 0) ? limit_scan_index : scans.size();
  GraphAdapter adapter(this, limit);
  kd_tree_t tree(2 /* dimension */, adapter, 5 /* leaf size */);
  tree.buildIndex();

  // Search KD-tree for neighbors
  const double query[2] = {scan->pose.x, scan->pose.y};
  std::vector<std::pair<unsigned, double>> matches;
  nanoflann::SearchParams params;
  tree.radiusSearch(query, dist, matches, params);

  std::vector<size_t> indicies;
  for (size_t i = 0; i < matches.size(); ++i)
  {
    indicies.push_back(matches[i].first);
  }
  return indicies;
}

void Graph::getMsg(visualization_msgs::msg::MarkerArray & msg, rclcpp::Time & t)
{
  // Publish nodes in red
  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = t;
  m.ns = "nodes";
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.scale.x = 0.08;
  m.scale.y = 0.08;
  m.scale.z = 0.08;
  m.color.r = 1.0;
  m.color.g = 0.0;
  m.color.b = 0.0;
  m.color.a = 1.0;

  for (auto & scan : scans)
  {
    // Publish
    m.id = scan->id;
    m.pose.position.x = scan->pose.x;
    m.pose.position.y = scan->pose.y;
    msg.markers.push_back(m);
  }

  // Publish odometry edges in blue
  m.header.frame_id = "map";
  m.header.stamp = t;
  m.ns = "edges";
  m.id = 0;
  m.type = visualization_msgs::msg::Marker::LINE_STRIP;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.scale.x = 0.04;
  m.scale.y = 0.04;
  m.scale.z = 0.04;

  for (auto & constraint : constraints)
  {
    auto & begin = scans[constraint->begin];
    auto & end = scans[constraint->end];
    m.points.resize(2);
    m.points[0].x = begin->pose.x;
    m.points[0].y = begin->pose.y;
    m.points[1].x = end->pose.x;
    m.points[1].y = end->pose.y;
    if (constraint->switchable)
    {
      m.color.r = 0.0;
      m.color.g = 1.0;
      m.color.b = 0.0;
      m.color.a = 1.0;
    }
    else
    {
      m.color.r = 0.0;
      m.color.g = 0.0;
      m.color.b = 1.0;
      m.color.a = 1.0;
    }
    msg.markers.push_back(m);
    ++m.id;
  }
}

}  // namespace ndt_2d
