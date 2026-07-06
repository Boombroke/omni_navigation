#!/usr/bin/env python3
# 仿真专用 GT 定位中继: 用 Gazebo 真值里程计 chassis_odometry_gt 替代仿真里退化的
# Point-LIO/odom_bridge, 在仿真链路里提供干净无漂移的定位与 MPPI 速度反馈。
# 仅在仿真 launch 中启用 (odom_bridge 被 enable_odom_bridge:=False 门控关闭), 实车不受影响。
#
# 复刻 odom_bridge 的对外契约, 使 MPPI/velocity_smoother/terrain 行为与实车一致 (同链路):
#   - TF odom -> base_footprint (2D 约束; spawn 相对, 使 odom 原点=出生点, 与 Point-LIO 约定一致)
#   - odometry        (odom -> robot_base_frame; 供 fake_vel_transform/velocity_smoother)
#   - chassis_odometry(odom -> base_frame; twist 线速度在 odom 惯性轴, 供 MPPI 速度反馈)
#   - registered_scan / lidar_odometry (透传 Point-LIO cloud_registered): terrain 感知链路必需,
#     否则 pointcloud_to_laserscan 无输入 -> slam_toolbox 无 /map -> costmap static_layer 阻塞
#     ("no map received") -> planner 无有效代价图 -> 不发 cmd_vel。定位用 GT (干净), 感知仍走
#     sim 雷达 (sim Point-LIO 位姿噪声只轻微影响 terrain 标记, 已被 min_obstacle_intensity 抑制)。
#
# 关键: chassis_odometry_gt 的 pose 为 Gazebo 世界系绝对值 (原点在世界原点, 非出生点),
# twist 线速度在世界系。本节点以首帧为 init, 输出 init^-1 * gt (spawn 相对), 并把 twist
# 旋回 odom 轴系; 出生 yaw=0 时为恒等, 但按 init yaw 通用处理。

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster


def yaw_from_quat(z: float, w: float) -> float:
    # 平面 (roll=pitch=0) 下的 yaw
    return 2.0 * math.atan2(z, w)


class ChassisOdomRelay(Node):
    def __init__(self):
        super().__init__("chassis_odom_relay")

        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_footprint").value
        self.robot_base_frame = self.declare_parameter(
            "robot_base_frame", "gimbal_yaw"
        ).value
        self.lidar_frame = self.declare_parameter("lidar_frame", "front_mid360").value
        self.gt_topic = self.declare_parameter(
            "gt_odom_topic", "chassis_odometry_gt"
        ).value
        self.registered_scan_topic = self.declare_parameter(
            "registered_scan_topic", "cloud_registered"
        ).value
        publish_rate = self.declare_parameter("publish_rate_hz", 100.0).value

        # spawn 相对基准 (首帧 GT)
        self._init_x = 0.0
        self._init_y = 0.0
        self._init_yaw = 0.0
        self._init_cos = 1.0
        self._init_sin = 0.0
        self._have_init = False
        self._last_gt = None

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odometry_pub = self.create_publisher(Odometry, "odometry", 2)
        self.chassis_odometry_pub = self.create_publisher(
            Odometry, "chassis_odometry", 2
        )
        self.registered_scan_pub = self.create_publisher(
            PointCloud2, "registered_scan", 5
        )
        self.lidar_odometry_pub = self.create_publisher(Odometry, "lidar_odometry", 5)

        self.create_subscription(
            Odometry, self.gt_topic, self._gt_cb, qos_profile_sensor_data
        )
        self.create_subscription(
            PointCloud2,
            self.registered_scan_topic,
            self._scan_cb,
            qos_profile_sensor_data,
        )

        self.create_timer(1.0 / publish_rate, self._tick)
        self.get_logger().info(
            f"chassis_odom_relay ready: gt={self.gt_topic} -> TF {self.odom_frame}"
            f"->{self.base_frame} + chassis_odometry (sim GT localization)"
        )

    def _gt_cb(self, msg: Odometry):
        if not self._have_init:
            p = msg.pose.pose
            self._init_x = p.position.x
            self._init_y = p.position.y
            self._init_yaw = yaw_from_quat(p.orientation.z, p.orientation.w)
            self._init_cos = math.cos(self._init_yaw)
            self._init_sin = math.sin(self._init_yaw)
            self._have_init = True
            self.get_logger().info(
                f"init GT pose captured: world=({self._init_x:.3f},{self._init_y:.3f}) "
                f"yaw={self._init_yaw:.3f} -> odom origin"
            )
        self._last_gt = msg

    def _rel_pose(self, msg: Odometry):
        # 把世界系 GT 位姿转成出生点相对 (odom 系): rel = init^-1 * gt
        p = msg.pose.pose
        dx = p.position.x - self._init_x
        dy = p.position.y - self._init_y
        c, s = self._init_cos, self._init_sin
        rel_x = c * dx + s * dy
        rel_y = -s * dx + c * dy
        gyaw = yaw_from_quat(p.orientation.z, p.orientation.w)
        rel_yaw = gyaw - self._init_yaw
        return rel_x, rel_y, rel_yaw

    def _rel_twist(self, msg: Odometry):
        # 世界系线速度旋回 odom 轴系 (yaw rate 与系无关)
        t = msg.twist.twist
        c, s = self._init_cos, self._init_sin
        vx = c * t.linear.x + s * t.linear.y
        vy = -s * t.linear.x + c * t.linear.y
        return vx, vy, t.angular.z

    def _tick(self):
        if not self._have_init or self._last_gt is None:
            return
        msg = self._last_gt
        stamp = msg.header.stamp
        rel_x, rel_y, rel_yaw = self._rel_pose(msg)
        vx, vy, wz = self._rel_twist(msg)
        qz = math.sin(rel_yaw * 0.5)
        qw = math.cos(rel_yaw * 0.5)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = rel_x
        tf.transform.translation.y = rel_y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        # chassis_odometry: odom -> base_frame, twist 在 odom 惯性轴 (供 MPPI)
        chassis = Odometry()
        chassis.header.stamp = stamp
        chassis.header.frame_id = self.odom_frame
        chassis.child_frame_id = self.base_frame
        chassis.pose.pose.position.x = rel_x
        chassis.pose.pose.position.y = rel_y
        chassis.pose.pose.orientation.z = qz
        chassis.pose.pose.orientation.w = qw
        chassis.twist.twist.linear.x = vx
        chassis.twist.twist.linear.y = vy
        chassis.twist.twist.angular.z = wz
        self.chassis_odometry_pub.publish(chassis)

        # odometry: odom -> robot_base_frame (仿真无云台独立运动, gimbal_yaw ~= chassis)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.robot_base_frame
        odom.pose.pose.position.x = rel_x
        odom.pose.pose.position.y = rel_y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        self.odometry_pub.publish(odom)

    def _scan_cb(self, msg: PointCloud2):
        # 透传 Point-LIO 已配准点云为 registered_scan (odom 系, 供 terrain_analysis);
        # 同步发 lidar_odometry (odom -> lidar_frame, 供 terrain_analysis_ext)。
        # 点云已在 odom 原点系, 但源 frame_id 为孤儿 (camera_init), 未接入命名空间化 TF 树 ->
        # pointcloud_to_laserscan/terrain 变换失败 -> slam 偶发缺 scan, RViz 地图错位。
        # 显式改回 odom_frame (与 odom_bridge 契约一致), 使点云 frame 落在有效 TF 树上。
        msg.header.frame_id = self.odom_frame
        self.registered_scan_pub.publish(msg)
        if not self._have_init or self._last_gt is None:
            return
        rel_x, rel_y, rel_yaw = self._rel_pose(self._last_gt)
        lo = Odometry()
        lo.header.stamp = msg.header.stamp
        lo.header.frame_id = self.odom_frame
        lo.child_frame_id = self.lidar_frame
        lo.pose.pose.position.x = rel_x
        lo.pose.pose.position.y = rel_y
        lo.pose.pose.orientation.z = math.sin(rel_yaw * 0.5)
        lo.pose.pose.orientation.w = math.cos(rel_yaw * 0.5)
        self.lidar_odometry_pub.publish(lo)


def main():
    rclpy.init()
    node = ChassisOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
