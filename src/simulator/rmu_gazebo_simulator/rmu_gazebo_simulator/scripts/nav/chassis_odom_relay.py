#!/usr/bin/env python3
# 仿真专用 GT 定位中继: 用 Gazebo 真值里程计 chassis_odometry_gt 替代仿真里退化的
# Point-LIO/odom_bridge, 在仿真链路里提供干净无漂移的定位与 MPPI 速度反馈。
# 仅在仿真 launch 中启用 (odom_bridge 被 enable_odom_bridge:=False 门控关闭), 实车不受影响。
#
# 复刻 odom_bridge 的对外契约, 使 MPPI/velocity_smoother/terrain 行为与实车一致 (同链路):
#   - TF odom -> base_footprint (2D 约束; spawn 相对, 使 odom 原点=出生点, 与 Point-LIO 约定一致)
#   - odometry        (odom -> robot_base_frame; 供 fake_vel_transform/velocity_smoother)
#   - chassis_odometry(odom -> base_frame; twist 线速度在 odom 惯性轴, 供 MPPI 速度反馈)
#   - registered_scan / lidar_odometry: terrain 感知链路必需 (否则 p2l 无输入 -> slam 无 /map ->
#     costmap static_layer 阻塞)。registered_scan 由原始 sim 雷达 (velodyne_points, sensor 系)
#     经 GT 的 odom<-sensor TF 真变换到 odom 得到 -> 与 GT 机器人位姿严格一致, 免疫 Point-LIO 漂移
#     (旧实现仅改标 Point-LIO camera_init 云的 frame_id 为 odom 不变换坐标, 运动后随 Point-LIO
#     漂移偏离 GT, 使 RViz 点云与机器人/地图错位)。
#
# 关键: chassis_odometry_gt 的 pose 为 Gazebo 世界系绝对值 (原点在世界原点, 非出生点),
# twist 线速度在世界系。本节点以首帧为 init, 输出 init^-1 * gt (spawn 相对), 并把 twist
# 旋回 odom 轴系; 出生 yaw=0 时为恒等, 但按 init yaw 通用处理。

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
        self.raw_scan_topic = self.declare_parameter(
            "raw_scan_topic", "velodyne_points"
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
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
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
            self.raw_scan_topic,
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

    @staticmethod
    def _quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
        # 单位四元数 -> 3x3 旋转矩阵 (点云批量旋转用)
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ]
        )

    def _scan_cb(self, msg: PointCloud2):
        # 用 GT 定位把原始仿真雷达点云 (sensor 系) 真变换到 odom 系发 registered_scan:
        # odom<-base 由本中继按 GT 广播, base<-sensor 是 URDF 静态外参, 故 tf 查得的 odom<-sensor
        # 完全由 GT 决定 -> 感知点云严格贴合 GT 机器人位姿, 免疫 sim Point-LIO 漂移。
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame, msg.header.frame_id, msg.header.stamp,
                Duration(seconds=0.05),
            )
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, msg.header.frame_id, Time()
                )
            except TransformException:
                return

        pts = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )
        pts = pts[np.isfinite(pts[:, :3]).all(axis=1)]
        if pts.shape[0] == 0:
            return
        tr = tf.transform.translation
        q = tf.transform.rotation
        rot = self._quat_to_rot(q.x, q.y, q.z, q.w)
        xyz = pts[:, :3].astype(np.float64) @ rot.T + np.array([tr.x, tr.y, tr.z])

        cloud = np.zeros(
            xyz.shape[0],
            dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4")],
        )
        cloud["x"] = xyz[:, 0]
        cloud["y"] = xyz[:, 1]
        cloud["z"] = xyz[:, 2]
        cloud["intensity"] = pts[:, 3]
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.header.frame_id = self.odom_frame
        self.registered_scan_pub.publish(point_cloud2.create_cloud(msg.header, fields, cloud))

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
