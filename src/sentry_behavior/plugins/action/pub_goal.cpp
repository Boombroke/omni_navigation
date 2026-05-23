// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "sentry_behavior/plugins/action/pub_goal.hpp"

#include <cmath>

namespace sentry_behavior
{

PubGoalAction::PubGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: RosTopicPubNode<geometry_msgs::msg::PoseStamped>(name, conf, params)
{
}

bool PubGoalAction::setMessage(geometry_msgs::msg::PoseStamped & msg)
{
  auto res_x = getInput<float>("goal_pose_x");
  auto res_y = getInput<float>("goal_pose_y");
  if (!res_x) {
    throw BT::RuntimeError("error reading port [goal_pose_x]: ", res_x.error());
  }
  if (!res_y) {
    throw BT::RuntimeError("error reading port [goal_pose_y]: ", res_y.error());
  }
  float yaw = 0.0f;
  getInput<float>("goal_pose_yaw", yaw);

  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = res_x.value();
  msg.pose.position.y = res_y.value();
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(yaw * 0.5f);
  msg.pose.orientation.w = std::cos(yaw * 0.5f);
  return true;
}

BT::PortsList PubGoalAction::providedPorts()
{
  BT::PortsList additional_ports = {
    BT::InputPort<float>("goal_pose_x"),
    BT::InputPort<float>("goal_pose_y"),
    BT::InputPort<float>("goal_pose_yaw", 0.0f, "目标航向 (rad), 默认 0"),
  };
  return providedBasicPorts(additional_ports);
}

}  // namespace sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(sentry_behavior::PubGoalAction, "PubGoal");
