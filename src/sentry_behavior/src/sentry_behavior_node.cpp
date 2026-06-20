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

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "sentry_behavior/behavior_machine.hpp"
#include "sentry_behavior/goal_publisher.hpp"
#include "sentry_behavior/referee_snapshot.hpp"
#include "sentry_behavior/strategies.hpp"

namespace sentry_behavior
{

class SentryBehaviorNode : public rclcpp::Node
{
public:
  explicit SentryBehaviorNode(const rclcpp::NodeOptions & options)
  : rclcpp::Node("sentry_behavior_node", options)
  {
    const std::string strategy_name = declare_parameter<std::string>("strategy", "rmuc_defend");
    const double tick_frequency = declare_parameter<double>("tick_frequency", 20.0);

    StrategyParams params;
    params.rmuc_hp_min = declare_parameter<int>("rmuc_hp_min", params.rmuc_hp_min);
    params.attack_hp_min = declare_parameter<int>("attack_hp_min", params.attack_hp_min);
    params.patrol_hp_min = declare_parameter<int>("patrol_hp_min", params.patrol_hp_min);
    params.ammo_min = declare_parameter<int>("ammo_min", params.ammo_min);
    params.outpost_hp_min = declare_parameter<int>("outpost_hp_min", params.outpost_hp_min);

    if (tick_frequency <= 0.0) {
      throw std::runtime_error("tick_frequency must be > 0");
    }

    Strategy strategy = make_strategy(strategy_name, params);

    goal_pub_ = std::make_unique<GoalPublisher>(this, "/goal_pose");
    machine_ = std::make_unique<BehaviorMachine>(std::move(strategy), goal_pub_.get());

    const auto qos = rclcpp::QoS(10);
    game_sub_ = create_subscription<rm_interfaces::msg::GameStatus>(
      "referee/game_status", qos,
      [this](const rm_interfaces::msg::GameStatus::SharedPtr msg) {
        snapshot_.progress = msg->game_progress;
        snapshot_.remain = msg->stage_remain_time;
        snapshot_.game_valid = true;
      });
    robot_sub_ = create_subscription<rm_interfaces::msg::RobotStatus>(
      "referee/robot_status", qos,
      [this](const rm_interfaces::msg::RobotStatus::SharedPtr msg) {
        snapshot_.hp = msg->current_hp;
        snapshot_.ammo = msg->projectile_allowance_17mm;
        snapshot_.robot_valid = true;
      });
    hp_sub_ = create_subscription<rm_interfaces::msg::GameRobotHP>(
      "referee/all_robot_hp", qos,
      [this](const rm_interfaces::msg::GameRobotHP::SharedPtr msg) {
        snapshot_.outpost_hp = msg->ally_outpost_hp;
        snapshot_.outpost_valid = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / tick_frequency);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {
        Ctx ctx{snapshot_, now()};
        machine_->tick(ctx);
      });

    RCLCPP_INFO(
      get_logger(), "sentry_behavior_node ready, strategy=%s", strategy_name.c_str());
  }

private:
  RefereeSnapshot snapshot_;
  std::unique_ptr<GoalPublisher> goal_pub_;
  std::unique_ptr<BehaviorMachine> machine_;
  rclcpp::Subscription<rm_interfaces::msg::GameStatus>::SharedPtr game_sub_;
  rclcpp::Subscription<rm_interfaces::msg::RobotStatus>::SharedPtr robot_sub_;
  rclcpp::Subscription<rm_interfaces::msg::GameRobotHP>::SharedPtr hp_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace sentry_behavior

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sentry_behavior::SentryBehaviorNode>(rclcpp::NodeOptions());
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("sentry_behavior_node"), "fatal: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
