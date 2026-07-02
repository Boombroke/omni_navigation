# ign_sim_pointcloud_tool

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)

## 简介

`ign_sim_pointcloud_tool` 将 Gazebo Harmonic 仿真中 Livox Mid360 发出的原始点云（`PointXYZ` 格式）转换为 Point-LIO 所需的 Velodyne 格式（`PointXYZIRT`），补充 `ring` 和 `time` 字段后发布到 `velodyne_points`。

转换逻辑：
- 根据点的仰角（`atan2(z, sqrt(x²+y²))`）和参数 `ang_bottom` / `ang_res_y` 计算 `ring`（行号），超出 `[0, n_scan)` 范围的点丢弃
- per-point 时间戳：`time = (point_id % horizon_scan) * 0.1 / horizon_scan`，**单位为秒**

## 关键注意事项

**`time` 字段单位是秒。** Point-LIO 的 `preprocess.timestamp_unit` 必须设为 `0`（秒）。若误设为 `2`（微秒），Point-LIO 会将每帧内各点的时间偏移放大 10^6 倍，导致运动去畸变完全失效，高速运动时里程计漂移严重。

```yaml
# point_lio 仿真配置（正确）
preprocess:
  timestamp_unit: 0  # 0=秒，勿改为 2（微秒）
```

## 订阅话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `pcd_topic` | `sensor_msgs/msg/PointCloud2` | Gazebo Mid360 原始点云，默认 `livox/lidar` |

## 发布话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `velodyne_points` | `sensor_msgs/msg/PointCloud2` | 转换后的 PointXYZIRT 格式点云 |

## 参数

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `pcd_topic` | string | `livox/lidar` | 输入点云话题 |
| `n_scan` | int | `32` | 线数（垂直方向分辨率行数） |
| `horizon_scan` | int | `1875` | 水平方向每圈点数，用于计算 per-point time |
| `ang_bottom` | float | `7.0` | 垂直角度下边界（度），用于 ring 计算 |
| `ang_res_y` | float | `1.0` | 垂直角分辨率（度/行），用于 ring 计算 |
