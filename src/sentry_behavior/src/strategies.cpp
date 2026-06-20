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

#include "sentry_behavior/strategies.hpp"

#include <stdexcept>

namespace sentry_behavior
{
namespace
{

Strategy make_rmuc_defend(const StrategyParams & p)
{
  const int hp_min = p.rmuc_hp_min;
  const int ammo_min = p.ammo_min;
  Strategy s;
  s.name = "rmuc_defend";
  s.states.push_back(
    TacticalState{
      "DEFEND",
      [hp_min, ammo_min](const Ctx & c) {
        return c.referee.robot_valid && c.referee.hp >= hp_min && c.referee.ammo >= ammo_min;
      },
      GoalCmd{3.71, -0.61, 0.0}});
  s.states.push_back(
    TacticalState{
      "SUPPLY",
      [](const Ctx &) {return true;},
      GoalCmd{-0.27, -3.94, 0.0}});
  return s;
}

}  // namespace

Strategy make_strategy(const std::string & name, const StrategyParams & p)
{
  if (name == "rmuc_defend") {
    return make_rmuc_defend(p);
  }
  throw std::runtime_error("unknown strategy: " + name);
}

}  // namespace sentry_behavior
