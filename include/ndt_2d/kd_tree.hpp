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

#ifndef NDT_2D__KD_TREE_HPP_
#define NDT_2D__KD_TREE_HPP_

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <utility>

namespace ndt_2d
{

/**
 * @brief Unbalanced KD-Tree used for KLD computations
 *
 * Graph search is done using nanoflann, but nanoflann does not support easily
 * building an incremental KD-Tree, as required by KLD. It also lacks a counter
 * of how many leaf nodes exist, which is the primary use case for KLD.
 */
template <typename T>
class KDTree
{
  struct Node
  {
    Node(int key[], T & value)
    {
      for (size_t i = 0; i < 3; ++i)
      {
        this->key[i] = key[i];
      }
      this->value = value;

      this->pivot = -1;
      this->child[0] = nullptr;
      this->child[1] = nullptr;
    }

    // Key in discrete space (x, y, theta)
    int key[3];
    // If a leaf node: value stored, if not a leaf, this is the pivot value
    T value;
    // To better balance the tree, dynamically select the pivot axis
    int pivot;

    std::shared_ptr<Node> child[2];
  };
  using NodePtr = std::shared_ptr<Node>;

public:
  /**
   * @brief Create an empty KD Tree
   * @param size_x Size in X direction of each leaf node, in meters.
   * @param size_y Size in Y direction of each leaf node, in meters.
   * @param size_th Size in angular rotation of each leaf node, in radians.
   * @param size_reserve Size to reserve so we don't have to re-alloc.
   *
   * For KLD adaptive resampling, Probabilistic Robotics recommends
   * a leaf node size of 0.5m x 0.5m x 15 degrees.
   */
  KDTree(double size_x, double size_y, double size_th, size_t size_reserve)
  : cell_size_(size_x, size_y, size_th),
    leaf_count_(0)
  {
    nodes_.reserve(size_reserve);
  }

  /**
   *
   */
  void insert(Eigen::Vector3d & cont_key, T & value)
  {
    int key[3];
    key[0] = static_cast<int>(cont_key(0) / cell_size_(0));
    key[1] = static_cast<int>(cont_key(1) / cell_size_(1));
    key[2] = static_cast<int>(cont_key(2) / cell_size_(2));

    if (nodes_.empty())
    {
      // New node is a leaf
      NodePtr node = std::make_shared<Node>(key, value);
      nodes_.push_back(node);
      ++leaf_count_;
    }
    else
    {
      // Copy of shared_ptr is valid even during re-alloc
      NodePtr parent = nodes_[0];
      NodePtr nullnode;
      _insert(parent, nullnode, key, value);
    }
  }

  size_t getLeafCount()
  {
    return leaf_count_;
  }

  void clear()
  {
    nodes_.clear();
    leaf_count_ = 0;
  }

private:
  void _insert(NodePtr &  parent, NodePtr & node, int key[], T value)
  {
    int * k = (node) ? node->key : key;
    if (parent->pivot >= 0)
    {
      // Parent node is NOT a leaf node
      size_t idx = (k[parent->pivot] < parent->value) ? 0 : 1;
      // Insert node into proper subtree
      _insert(parent->child[idx], node, key, value);
    }
    else
    {
      // Parent node is a leaf node
      // First check for equality
      if (parent->key[0] == k[0] && parent->key[1] == k[1] && parent->key[2] == k[2])
      {
        // Update value
        parent->value += value;
        // Return created node
        node = parent;
        return;
      }

      // Find the axis with largest variation and compute the pivot value
      int max_variation = 0;
      for (size_t i = 0; i < 3; ++i)
      {
        int variation = abs(parent->key[i] - k[i]);
        if (variation > max_variation)
        {
          parent->pivot = i;
          max_variation = variation;
        }
      }

      // Create new leaf nodes to carry the value of parent
      parent->child[0] = std::make_shared<Node>(parent->key, parent->value);
      parent->child[1] = std::make_shared<Node>(key, value);
      nodes_.push_back(parent->child[0]);
      nodes_.push_back(parent->child[1]);

      // Return created node
      node = parent->child[1];

      // Only one new leaf node - the other is just moved
      ++leaf_count_;

      // Change the value of the non-leaf node to the pivot value
      parent->value = (parent->child[0]->key[parent->pivot] +
                       parent->child[1]->key[parent->pivot]) / 2.0;

      // Put children in correct order
      if (parent->child[1]->key[parent->pivot] < parent->value)
      {
        std::swap(parent->child[0], parent->child[1]);
      }
    }
  }

  Eigen::Vector3d cell_size_;
  size_t leaf_count_;

  // nodes_[0] is always root
  std::vector<NodePtr> nodes_;
};

}  // namespace ndt_2d

#endif  // NDT_2D__KD_TREE_HPP_
