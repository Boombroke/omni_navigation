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

#include "sentry_behavior/behavior_machine.hpp"

#include <utility>

namespace sentry_behavior
{

const TacticalState & Strategy::evaluate(const Ctx & c) const
{
  for (const auto & s : states) {
    if (s.active && s.active(c)) {
      return s;
    }
  }
  return states.back();
}

BehaviorMachine::BehaviorMachine(
  Strategy strategy, GoalPublisher * goal, bool follow_enable, double follow_staleness_sec)
: strategy_(std::move(strategy)),
  goal_(goal),
  follow_enable_(follow_enable),
  follow_staleness_sec_(follow_staleness_sec)
{
}

bool BehaviorMachine::game_gate(const Ctx & c)
{
  return c.referee.game_valid && c.referee.progress == 4 &&
         c.referee.remain >= 0 && c.referee.remain <= 420;
}

void BehaviorMachine::enter_wait_start()
{
  phase_ = Phase::WAIT_START;
  tactical_leaf_.clear();
  current_goal_.reset();
  goal_->reset();
}

void BehaviorMachine::tick(const Ctx & c)
{
  const bool in = game_gate(c);

  if (phase_ == Phase::IN_MATCH && !in) {
    enter_wait_start();
  } else if (phase_ == Phase::WAIT_START && in) {
    phase_ = Phase::IN_MATCH;
  }

  if (phase_ == Phase::WAIT_START) {
    tactical_leaf_.clear();
    current_goal_.reset();
    return;
  }

  // FOLLOW: 最高优先父 guard, 动态目标 (节点已算好 map 系 standoff 目标), 抢占战术层。
  // 仅当 启用 && 目标有效 && 未过期 才触发; 否则落回原战术 guard 表, 现有策略与不变量不受影响。
  if (follow_enable_ && c.target.valid) {
    const double age = (c.now.nanoseconds() - c.target.stamp.nanoseconds()) * 1e-9;
    if (age >= 0.0 && age <= follow_staleness_sec_) {
      tactical_leaf_ = "FOLLOW";
      current_goal_ = c.target.goal;
      goal_->request(c.target.goal);
      return;
    }
  }

  const TacticalState & s = strategy_.evaluate(c);
  tactical_leaf_ = s.id;
  current_goal_ = s.goal;
  if (s.goal) {
    goal_->request(*s.goal);
  }
}

std::vector<std::string> BehaviorMachine::tactical_state_ids() const
{
  std::vector<std::string> ids;
  ids.reserve(strategy_.states.size() + 1);
  if (follow_enable_) {
    ids.push_back("FOLLOW");
  }
  for (const auto & s : strategy_.states) {
    ids.push_back(s.id);
  }
  return ids;
}

std::vector<std::string> BehaviorMachine::active_path() const
{
  if (phase_ == Phase::WAIT_START) {
    return {"WAIT_START"};
  }
  if (tactical_leaf_.empty()) {
    return {"IN_MATCH"};
  }
  return {"IN_MATCH", tactical_leaf_};
}

}  // namespace sentry_behavior
