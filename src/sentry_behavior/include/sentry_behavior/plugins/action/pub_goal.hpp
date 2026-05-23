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

#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

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
class PubGoalAction : public BT::RosTopicPubNode<geometry_msgs::msg::PoseStamped>
{
public:
  PubGoalAction(
    const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params);

  static BT::PortsList providedPorts();

  bool setMessage(geometry_msgs::msg::PoseStamped & goal) override;
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__PLUGINS__ACTION__PUB_GOAL_HPP_
