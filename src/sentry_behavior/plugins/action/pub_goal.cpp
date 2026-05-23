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
: BT::SyncActionNode(name, conf), node_(params.nh), last_publish_time_(0, 0, RCL_ROS_TIME)
{
}

BT::NodeStatus PubGoalAction::tick()
{
  std::string topic_name;
  if (!getInput<std::string>("topic_name", topic_name) || topic_name.empty()) {
    throw BT::RuntimeError("missing port [topic_name]");
  }

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

  const double new_x = static_cast<double>(res_x.value());
  const double new_y = static_cast<double>(res_y.value());

  if (!publisher_ || current_topic_ != topic_name) {
    publisher_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
    current_topic_ = topic_name;
  }

  const rclcpp::Time now = node_->now();
  if (
    has_last_ && new_x == last_x_ && new_y == last_y_ &&
    (now - last_publish_time_).seconds() < kThrottleSeconds) {
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = now;
  msg.header.frame_id = "map";
  msg.pose.position.x = new_x;
  msg.pose.position.y = new_y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(yaw * 0.5f);
  msg.pose.orientation.w = std::cos(yaw * 0.5f);

  publisher_->publish(msg);

  last_x_ = new_x;
  last_y_ = new_y;
  last_publish_time_ = now;
  has_last_ = true;
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList PubGoalAction::providedPorts()
{
  return {
    BT::InputPort<std::string>("topic_name", "/goal_pose", "PoseStamped topic to publish"),
    BT::InputPort<float>("goal_pose_x"),
    BT::InputPort<float>("goal_pose_y"),
    BT::InputPort<float>("goal_pose_yaw", 0.0f, "目标航向 (rad), 默认 0"),
  };
}

}  // namespace sentry_behavior

#include "behaviortree_ros2/plugins.hpp"
CreateRosNodePlugin(sentry_behavior::PubGoalAction, "PubGoal");
