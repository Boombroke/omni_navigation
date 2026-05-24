//Licensed under the Apache License, Version 2.0 (the "License");
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

#ifndef SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GOAL_HPP_
#define SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GOAL_HPP_

#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/ros_node_params.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace sentry_behavior
{

// PubGoal: 一次性向 /goal_pose 发布 PoseStamped 后立刻 SUCCESS, 不等
// 任何 nav 结果 (fire-and-forget). 用于 BT 不愿意被 nav 阻塞的场景:
// 比如设点后立刻去做视觉锁敌 / 监听其他状态.
//
// 与 NavigateTo (RosActionNode<NavigateToPose>) 的差别:
// - NavigateTo 调用 nav2 navigate_to_pose action, 阻塞等真实 SUCCESS/FAILURE,
//   halt 时 cancel goal
// - PubGoal 只 publish topic, 不知道 nav 是否到点 / 失败 / 被打断,
//   halt 时不会 cancel 任何已发出的 goal (topic 没 handle)
//
// nav2 bt_navigator 订阅 /goal_pose 内部触发 navigate_to_pose action;
// 因此与 NavigateTo 在 nav2 侧最终是同一条物理路径. 谁后到 nav2 谁覆盖.
//
// 节流: 与 /goal_pose 当前 in-flight goal 相同的 (x,y) 不重复 publish. nav2 拿到
// goal 后自己规划+跟随, 不需要 BT 反复重发. 重发会触发 bt_navigator preemption +
// ComputePathToPose 重启, 中断 controller_server, 表现为路径卡顿.
// xy 变化时立刻 publish, 切换攻击点/补给点响应性不受影响.
//
// 实现要点: 节流状态用进程级 static 共享, 而不是节点实例级. 因为同一棵树有多个
// PubGoal 节点 (攻击点 / 补给点 / ...), 它们在分支切换时要协作: 攻击 -> 补给
// -> 攻击 这种回切, 第二个攻击节点 tick 时必须 publish (因为 /goal_pose 实际
// 在跟随补给点). 节点级 last_x_/y_ 会让"回切"被错误屏蔽 (实测分支已切但 nav
// 没收到新 goal 一直跟着旧的).
//
// **永远返回 SUCCESS, 即便跳过 publish** —— 历史版本继承 RosTopicPubNode 时
// setMessage return false 被基类翻译为 FAILURE, 让上层 Sequence FAILURE 整树退出,
// 是 BT 每秒重启的根因.
class PubGoalAction : public BT::SyncActionNode
{
public:
  PubGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
  std::string current_topic_;
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GOAL_HPP_
