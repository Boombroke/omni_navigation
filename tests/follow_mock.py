#!/usr/bin/env python3
# Follow-mode mock: 发布 map->chassis TF + 30Hz 运动 TrackGoal, 抓 /goal_pose 验证跟随效果。
# 物体绕机器人(原点)半径 2.5m 转圈; 期望 goal 维持 standoff(~1.5m), 并随物体移动;
# 5s 后停发目标 -> staleness 触发 -> 回落 rmuc_defend 守点 (3.71,-0.61)。
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rm_interfaces.msg import GameStatus, RobotStatus, TrackGoal
from tf2_ros import TransformBroadcaster

STANDOFF = 1.5
RADIUS = 2.5
PERIOD = 6.0
FOLLOW_SEC = 5.0
TOTAL_SEC = 8.0


class FollowMock(Node):
    def __init__(self):
        super().__init__("follow_mock")
        self.game_pub = self.create_publisher(GameStatus, "referee/game_status", 10)
        self.robot_pub = self.create_publisher(RobotStatus, "referee/robot_status", 10)
        self.target_pub = self.create_publisher(TrackGoal, "vision/target_body", 10)
        self.tf_bc = TransformBroadcaster(self)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.t0 = time.time()
        self.target = (0.0, 0.0)
        self.samples = []
        self.create_timer(0.1, self.pub_referee)
        self.create_timer(0.02, self.pub_tf)
        self.create_timer(1.0 / 30.0, self.pub_target)

    def pub_referee(self):
        g = GameStatus()
        g.game_progress = 4
        g.stage_remain_time = 200
        self.game_pub.publish(g)
        r = RobotStatus()
        r.current_hp = 300
        r.projectile_allowance_17mm = 100
        self.robot_pub.publish(r)

    def pub_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "chassis"
        t.transform.rotation.w = 1.0
        self.tf_bc.sendTransform(t)

    def pub_target(self):
        dt = time.time() - self.t0
        if dt > FOLLOW_SEC:
            return  # 停发: 触发 staleness 回落
        w = 2.0 * math.pi / PERIOD
        self.target = (RADIUS * math.cos(w * dt), RADIUS * math.sin(w * dt))
        m = TrackGoal()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = "chassis"
        m.rel_x = float(self.target[0])
        m.rel_y = float(self.target[1])
        m.rel_yaw = 0.0
        m.valid = True
        self.target_pub.publish(m)

    def on_goal(self, msg):
        self.samples.append(
            (time.time() - self.t0, msg.pose.position.x, msg.pose.position.y, self.target[0], self.target[1])
        )


def main():
    rclpy.init()
    n = FollowMock()
    end = time.time() + TOTAL_SEC
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.02)

    follow = [s for s in n.samples if s[0] <= FOLLOW_SEC]
    print(f"total /goal_pose samples = {len(n.samples)}")

    ok = True
    if len(follow) >= 5:
        dists = [math.hypot(gx - tx, gy - ty) for _, gx, gy, tx, ty in follow]
        gxs = [s[1] for s in follow]
        gys = [s[2] for s in follow]
        mean_d = sum(dists) / len(dists)
        span = max(max(gxs) - min(gxs), max(gys) - min(gys))
        print(f"[FOLLOW] samples={len(follow)} standoff(goal->target) mean={mean_d:.2f} "
              f"(want ~{STANDOFF}) min={min(dists):.2f} max={max(dists):.2f}")
        print(f"[FOLLOW] goal x=[{min(gxs):.2f},{max(gxs):.2f}] y=[{min(gys):.2f},{max(gys):.2f}] span={span:.2f} (want >1.0 = tracking)")
        ok = ok and abs(mean_d - STANDOFF) < 0.4 and span > 1.0
        print("[FOLLOW] PASS" if (abs(mean_d - STANDOFF) < 0.4 and span > 1.0) else "[FOLLOW] FAIL")
    else:
        ok = False
        print(f"[FOLLOW] FAIL: only {len(follow)} goals during follow phase (follow not active?)")

    last = n.samples[-1] if n.samples else None
    if last and last[0] > FOLLOW_SEC:
        lx, ly = last[1], last[2]
        back = math.hypot(lx - 3.71, ly + 0.61) < 0.05
        print(f"[FALLBACK] last goal=({lx:.2f},{ly:.2f}) @t={last[0]:.1f}s want (3.71,-0.61) -> "
              f"{'PASS' if back else 'FAIL'}")
        ok = ok and back
    else:
        ok = False
        print("[FALLBACK] FAIL: no goal after target lost")

    print("=== MOCK FOLLOW: PASS ===" if ok else "=== MOCK FOLLOW: FAIL ===")
    n.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
