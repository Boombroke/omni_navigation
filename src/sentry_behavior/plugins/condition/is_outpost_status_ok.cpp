// Copyright 2025 Lihan Chen
//
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

#include "sentry_behavior/plugins/condition/is_outpost_status_ok.hpp"

namespace sentry_behavior
{

IsOutpostStatusOKCondition::IsOutpostStatusOKCondition(
  const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsOutpostStatusOKCondition::checkOutpostStatus, this), config)
{
}

BT::NodeStatus IsOutpostStatusOKCondition::checkOutpostStatus()
{
  int outpost_hp_min = 1;
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "GameRobotHP message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("outpost_hp_min", outpost_hp_min);

  const bool is_outpost_alive = (msg->ally_outpost_hp >= static_cast<uint16_t>(outpost_hp_min));
  return is_outpost_alive ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList IsOutpostStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_gameRobotHP}", "GameRobotHP port on blackboard"),
    BT::InputPort<int>(
      "outpost_hp_min", 1,
      "outpost HP < outpost_hp_min returns FAILURE (inclusive: hp >= outpost_hp_min OK)")};
}

}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsOutpostStatusOKCondition>("IsOutpostStatusOK");
}
