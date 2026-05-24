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
#include <mutex>
#include <unordered_map>

namespace sentry_behavior
{

namespace
{
// 进程级共享: 每个 topic 上次实际 publish 出去的 (x,y).
// 多个 PubGoal 节点 (攻击/补给/...) 在分支切换时通过它协调, 避免回切到老分支
// 时被节点自身的 last 状态屏蔽掉真正必要的 publish.
struct LastGoal
{
  double x{0.0};
  double y{0.0};
  bool has{false};
};
std::mutex g_last_goal_mtx;
std::unordered_map<std::string, LastGoal> g_last_goal_per_topic;

// 进程级共享 publisher: 所有 PubGoal 节点对同一 topic 复用同一个 publisher.
// 不复用会导致只有第一个被 tick 的 PubGoal 节点的 publisher 被 bt_navigator
// 早期订阅, 后续节点首次 tick 时新建的 publisher 因 DDS discovery 时序问题
// 第一条消息可能不达 (实测: subscription_count 报 1 但 bt_navigator 没收到).
std::mutex g_pub_mtx;
std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  g_pub_per_topic;
}  // namespace

PubGoalAction::PubGoalAction(
  const std::string & name, const BT::NodeConfig & conf, const BT::RosNodeParams & params)
: BT::SyncActionNode(name, conf), node_(params.nh)
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

  // 取/建进程级共享 publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub;
  {
    std::lock_guard<std::mutex> lk(g_pub_mtx);
    auto it = g_pub_per_topic.find(topic_name);
    if (it == g_pub_per_topic.end()) {
      pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic_name, 1);
      g_pub_per_topic[topic_name] = pub;
    } else {
      pub = it->second;
    }
  }
  publisher_ = pub;
  current_topic_ = topic_name;

  {
    std::lock_guard<std::mutex> lk(g_last_goal_mtx);
    auto & last = g_last_goal_per_topic[topic_name];
    if (last.has && new_x == last.x && new_y == last.y) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = node_->now();
  msg.header.frame_id = "map";
  msg.pose.position.x = new_x;
  msg.pose.position.y = new_y;
  msg.pose.position.z = 0.0;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = std::sin(yaw * 0.5f);
  msg.pose.orientation.w = std::cos(yaw * 0.5f);

  // 有订阅者才认为 publish "生效"; 否则保持 last 不变, 下个 tick 还会重发.
  // 处理启动时序: BT 服务端比 nav2 bt_navigator 早就绪时, 第一次 publish
  // 没有订阅者会丢, 但若我们 set has=true, 之后又被去重逻辑挡住永远不补发.
  const size_t subs = publisher_->get_subscription_count();
  publisher_->publish(msg);

  if (subs == 0) {
    RCLCPP_WARN(
      node_->get_logger(),
      "PubGoal[%s] published (%.2f, %.2f) but %s has 0 subscribers, will retry next tick",
      name().c_str(), new_x, new_y, topic_name.c_str());
    return BT::NodeStatus::SUCCESS;
  }

  {
    std::lock_guard<std::mutex> lk(g_last_goal_mtx);
    auto & last = g_last_goal_per_topic[topic_name];
    last.x = new_x;
    last.y = new_y;
    last.has = true;
  }
  RCLCPP_INFO(
    node_->get_logger(), "PubGoal[%s] published (%.2f, %.2f) -> %s",
    name().c_str(), new_x, new_y, topic_name.c_str());
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
