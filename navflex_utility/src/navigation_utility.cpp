/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  navigation_utility.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "navflex_utility/navigation_utility.h"

namespace navflex_utility
{

bool getRobotPose(
  const rclcpp_lifecycle::LifecycleNode::ConstSharedPtr & node,
  const TF & tf,
  const std::string & robot_frame,
  const std::string & global_frame,
  const rclcpp::Duration & timeout,
  geometry_msgs::msg::PoseStamped & robot_pose)
{
  geometry_msgs::msg::PoseStamped local_pose;
  local_pose.header.frame_id = robot_frame;
  local_pose.header.stamp = rclcpp::Time(0, 0, node->get_clock()->get_clock_type());
  local_pose.pose.orientation.w = 1.0;
  bool success = transformPose(
    node,
    tf,
    global_frame,
    timeout,
    local_pose,
    robot_pose);
  const rclcpp::Duration transformAge = node->now() - robot_pose.header.stamp;
  if (success && transformAge > timeout) {
    RCLCPP_WARN(
      node->get_logger(), "Most recent robot pose is %gs old (tolerance %gs)",
      transformAge.seconds(), timeout.seconds());
    return false;
  }
  return success;
}

/**
 * @brief Returns true, if the given quaternion is normalized.
 *
 * @param _q The quaternion to check.
 * @param _epsilon The epsilon (squared distance to 1).
 */
static bool isNormalized(const geometry_msgs::msg::Quaternion & _q, double _epsilon)
{
  const double sq_sum =
    std::pow(_q.x, 2) + std::pow(_q.y, 2) + std::pow(_q.z, 2) + std::pow(_q.w, 2);
  return std::abs(sq_sum - 1.) <= _epsilon;
}

bool transformPose(
  const rclcpp_lifecycle::LifecycleNode::ConstSharedPtr node,
  const TF & tf,
  const std::string & target_frame,
  const rclcpp::Duration & timeout,
  const geometry_msgs::msg::PoseStamped & in,
  geometry_msgs::msg::PoseStamped & out)
{
  // Note: The tf-library does not check if the input is well formed.
  if (!isNormalized(in.pose.orientation, 0.01)) {
    RCLCPP_WARN_STREAM(
      node->get_logger(),
      "The given quaterinon " << geometry_msgs::msg::to_yaml(
        in.pose.orientation) << " is not normalized");
    return false;
  }

  if (target_frame == in.header.frame_id) {
    out = in;
    return true;
  }

  std::string error_msg;

  bool success = tf.canTransform(
    target_frame,
    in.header.frame_id,
    in.header.stamp,
    timeout,
    &error_msg);

  if (!success) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), "Failed to look up transform from frame '" << in.header.frame_id
                                                                     << "' into frame '" << target_frame << "': " <<
        error_msg);
    return false;
  }

  try {
    tf.transform(in, out, target_frame);
  } catch (const TFException & ex) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), "Failed to transform pose from frame '" << in.header.frame_id << " ' into frame '"
                                                                  << target_frame << "' with exception: " <<
        ex.what());
    return false;
  }
  return true;
}

bool transformPoint(
  const rclcpp_lifecycle::LifecycleNode::ConstSharedPtr & node,
  const TF & tf,
  const std::string & target_frame,
  const rclcpp::Duration & timeout,
  const geometry_msgs::msg::PointStamped & in,
  geometry_msgs::msg::PointStamped & out)
{
  std::string error_msg;

  bool success = tf.canTransform(
    target_frame,
    in.header.frame_id,
    in.header.stamp,
    timeout,
    &error_msg);

  if (!success) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), "Failed to look up transform from frame '" << in.header.frame_id
                                                                     << "' into frame '" << target_frame << "': " <<
        error_msg);
    return false;
  }

  try {
    tf.transform(in, out, target_frame);
  } catch (const TFException & ex) {
    RCLCPP_WARN_STREAM(
      node->get_logger(), "Failed to transform point from frame '" << in.header.frame_id
                                                                   << " ' into frame '" << target_frame << "' with exception: " <<
        ex.what());
    return false;
  }
  return true;
}

double distance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2)
{
  const geometry_msgs::msg::Point & p1 = pose1.pose.position;
  const geometry_msgs::msg::Point & p2 = pose2.pose.position;
  const double dx = p1.x - p2.x;
  const double dy = p1.y - p2.y;
  const double dz = p1.z - p2.z;
  return sqrt(dx * dx + dy * dy + dz * dz);
}

double angle(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2)
{
  const geometry_msgs::msg::Quaternion & q1 = pose1.pose.orientation;
  const geometry_msgs::msg::Quaternion & q2 = pose2.pose.orientation;
  tf2::Quaternion rot1, rot2;
  tf2::fromMsg(q1, rot1);
  tf2::fromMsg(q2, rot2);
  return rot1.angleShortestPath(rot2);
}

} /* namespace navflex_utility */
