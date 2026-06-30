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

#ifndef SENTRY_BEHAVIOR__BEHAVIOR_MACHINE_HPP_
#define SENTRY_BEHAVIOR__BEHAVIOR_MACHINE_HPP_

#include <functional>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sentry_behavior/goal_publisher.hpp"
#include "sentry_behavior/referee_snapshot.hpp"

namespace sentry_behavior
{

struct TargetSnapshot
{
  bool valid = false;
  GoalCmd goal;            // 已变换到 map 系并应用 standoff 的跟随目标 (由节点算好)
  rclcpp::Time stamp;      // 目标测量时刻 (staleness 判定)
};

struct Ctx
{
  const RefereeSnapshot & referee;
  const TargetSnapshot & target;
  rclcpp::Time now;
};

struct TacticalState
{
  std::string id;
  std::function<bool(const Ctx &)> active;
  std::optional<GoalCmd> goal;
};

struct Strategy
{
  std::string name;
  std::vector<TacticalState> states;

  const TacticalState & evaluate(const Ctx & c) const;
};

enum class Phase { WAIT_START, IN_MATCH };

// 2 层分层状态机: 生命周期 (WAIT_START<->IN_MATCH) × 战术 guard 表.
// 生命周期父 guard 每 tick 先判, 复刻旧树 ReactiveFallback[Inverter(IsGameStatus), 主决策]:
// 比赛中 game gate 失效立即转 WAIT_START 并复位目标, 主决策被抢占.
class BehaviorMachine
{
public:
  BehaviorMachine(
    Strategy strategy, GoalPublisher * goal, bool follow_enable, double follow_staleness_sec);

  void tick(const Ctx & c);

  std::vector<std::string> active_path() const;
  std::vector<std::string> tactical_state_ids() const;
  const std::optional<GoalCmd> & current_goal() const {return current_goal_;}
  std::string tactical_leaf() const {return tactical_leaf_;}
  Phase phase() const {return phase_;}

private:
  static bool game_gate(const Ctx & c);
  void enter_wait_start();

  Strategy strategy_;
  GoalPublisher * goal_;
  bool follow_enable_;
  double follow_staleness_sec_;
  Phase phase_ = Phase::WAIT_START;
  std::string tactical_leaf_;
  std::optional<GoalCmd> current_goal_;
};

}  // namespace sentry_behavior

#endif  // SENTRY_BEHAVIOR__BEHAVIOR_MACHINE_HPP_
