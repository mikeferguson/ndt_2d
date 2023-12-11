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

#include <iostream>
#include <ndt_2d/constraint.hpp>

namespace ndt_2d
{

ConstraintPtr makeConstraint(const ScanPtr & from,
                             const ScanPtr & to,
                             Eigen::Matrix3d & covariance)
{
  ConstraintPtr constraint = std::make_shared<Constraint>();
  constraint->begin = from->getId();
  constraint->end = to->getId();
  // Get the delta in map coordinates
  double dx = to->getPose().x - from->getPose().x;
  double dy = to->getPose().y - from->getPose().y;
  // Convert dx/dy into the coordinate frame of begin->pose
  double costh = cos(from->getPose().theta);
  double sinth = sin(from->getPose().theta);
  constraint->transform(0) = costh * dx + sinth * dy;
  constraint->transform(1) = -sinth * dx + costh * dy;
  constraint->transform(2) = to->getPose().theta - from->getPose().theta;
  // Create information matrix
  constraint->information = covariance.inverse();
  // Default is not switchable
  constraint->switchable = false;
  return constraint;
}

}  // namespace ndt_2d
