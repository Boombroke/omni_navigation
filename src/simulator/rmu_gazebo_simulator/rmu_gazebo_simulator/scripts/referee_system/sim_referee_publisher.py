#!/usr/bin/env python3
# 仿真裁判发布器: 以 rm_interfaces 契约持续发布 referee/{game_status,robot_status,all_robot_hp},
# 驱动 sentry_behavior 状态机在仿真里真发 /goal_pose(仿真无真实裁判系统)。
# 参数化健康/补给场景, 供 P5b 闭环与离线回归复用。
# rclpy 直发, 规避 jazzy ros2 topic pub CLI 对带中文注释 rm_interfaces .msg 的 rosidl_adapter bug。

import rclpy
from rclpy.node import Node

from rm_interfaces.msg import GameRobotHP, GameStatus, RobotStatus


class SimRefereePublisher(Node):
    def __init__(self):
        super().__init__("sim_referee_publisher")

        self.game_progress = self.declare_parameter("game_progress", 4).value
        self.stage_remain_time = self.declare_parameter("stage_remain_time", 300).value
        self.current_hp = self.declare_parameter("current_hp", 300).value
        self.maximum_hp = self.declare_parameter("maximum_hp", 400).value
        self.projectile_allowance_17mm = self.declare_parameter(
            "projectile_allowance_17mm", 100
        ).value
        self.ally_outpost_hp = self.declare_parameter("ally_outpost_hp", 600).value
        self.countdown = self.declare_parameter("countdown", False).value
        rate_hz = self.declare_parameter("publish_rate_hz", 10.0).value

        self.game_pub = self.create_publisher(GameStatus, "referee/game_status", 10)
        self.robot_pub = self.create_publisher(RobotStatus, "referee/robot_status", 10)
        self.hp_pub = self.create_publisher(GameRobotHP, "referee/all_robot_hp", 10)

        self.create_timer(1.0 / rate_hz, self._tick)
        self.get_logger().info(
            "sim_referee_publisher ready: "
            f"progress={self.game_progress} hp={self.current_hp} "
            f"ammo={self.projectile_allowance_17mm}"
        )

    def _tick(self):
        game = GameStatus()
        game.game_progress = self.game_progress
        game.stage_remain_time = self.stage_remain_time
        game.behavior_state = 0
        self.game_pub.publish(game)

        robot = RobotStatus()
        robot.current_hp = self.current_hp
        robot.maximum_hp = self.maximum_hp
        robot.projectile_allowance_17mm = self.projectile_allowance_17mm
        self.robot_pub.publish(robot)

        hp = GameRobotHP()
        hp.ally_outpost_hp = self.ally_outpost_hp
        hp.ally_7_robot_hp = self.current_hp
        self.hp_pub.publish(hp)

        if self.countdown and self.stage_remain_time > 0:
            self.stage_remain_time -= 1


def main():
    rclpy.init()
    node = SimRefereePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
