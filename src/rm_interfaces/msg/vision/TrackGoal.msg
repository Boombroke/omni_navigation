# 跟随目标: 车体 (chassis) 系下的相对位姿, 由 rm_serial_driver 从串口解析后发布。
# header.frame_id 应为 chassis; header.stamp = 测量时刻 (供下游做 chassis->map 变换)。
std_msgs/Header header
float32 rel_x      # chassis 系前向相对位置 [m]
float32 rel_y      # chassis 系侧向相对位置 [m]
float32 rel_yaw    # 目标相对朝向 yaw [rad]
bool valid         # 是否存在有效跟踪目标
