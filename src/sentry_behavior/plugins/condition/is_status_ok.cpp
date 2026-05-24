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

#include "sentry_behavior/plugins/condition/is_status_ok.hpp"

namespace sentry_behavior
{

IsStatusOKCondition::IsStatusOKCondition(const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsStatusOKCondition::checkRobotStatus, this), config)
{
}

BT::NodeStatus IsStatusOKCondition::checkRobotStatus()
{
  int hp_min, ammo_min, ammo_max;
  auto msg = getInput<rm_interfaces::msg::RobotStatus>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "RobotStatus message is not available");
    return BT::NodeStatus::FAILURE;
  }

  getInput("hp_min", hp_min);
  getInput("ammo_min", ammo_min);
  getInput("ammo_max", ammo_max);

  // 阈值含等号语义: hp >= hp_min, ammo_min <= ammo <= ammo_max
  const bool is_hp_ok = (msg->current_hp >= hp_min);
  const bool is_ammo_ok = (msg->projectile_allowance_17mm >= ammo_min) &&
                          (msg->projectile_allowance_17mm <= ammo_max);
  const auto status = (is_hp_ok && is_ammo_ok) ? BT::NodeStatus::SUCCESS
                                               : BT::NodeStatus::FAILURE;
  // 限速 1Hz 打印当前判定, 便于诊断决策切换异常 (e.g. 血量已扣却没切补给).
  // Clock 用 static 保持 throttle 内部状态; 否则每次 make_shared 重置就失效.
  static rclcpp::Clock throttle_clock(RCL_STEADY_TIME);
  RCLCPP_INFO_THROTTLE(
    logger_, throttle_clock, 1000,
    "IsStatusOK[%s] hp=%d/%d ammo=%d/[%d,%d] -> %s",
    name().c_str(), msg->current_hp, hp_min,
    msg->projectile_allowance_17mm, ammo_min, ammo_max,
    status == BT::NodeStatus::SUCCESS ? "SUCCESS" : "FAILURE");
  return status;
}

BT::PortsList IsStatusOKCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::RobotStatus>(
      "key_port", "{@referee_robotStatus}", "RobotStatus port on blackboard"),
    BT::InputPort<int>("hp_min", 300, "HP < hp_min returns FAILURE (inclusive: hp >= hp_min OK)"),
    BT::InputPort<int>(
      "ammo_min", 0, "ammo < ammo_min returns FAILURE (inclusive: ammo >= ammo_min OK)"),
    BT::InputPort<int>(
      "ammo_max", 65535, "ammo > ammo_max returns FAILURE (inclusive: ammo <= ammo_max OK)")};
}
}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsStatusOKCondition>("IsStatusOK");
}
