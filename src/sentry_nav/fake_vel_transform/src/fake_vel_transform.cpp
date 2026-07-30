// Copyright 2025 Lihan Chen
//
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

#include "fake_vel_transform/fake_vel_transform.hpp"

#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace fake_vel_transform
{

FakeVelTransform::FakeVelTransform(const rclcpp::NodeOptions & options)
: Node("fake_vel_transform", options)
{
  RCLCPP_INFO(get_logger(), "Start FakeVelTransform!");

  this->declare_parameter<std::string>("robot_base_frame", "gimbal_yaw");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_yaw_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("cmd_spin_topic", "cmd_spin");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");
  this->declare_parameter<float>("init_spin_speed", 0.0);

  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("cmd_spin_topic", cmd_spin_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);
  this->get_parameter("init_spin_speed", spin_speed_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_spin_sub_ = this->create_subscription<example_interfaces::msg::Float32>(
    cmd_spin_topic_, 1, std::bind(&FakeVelTransform::cmdSpinCallback, this, std::placeholders::_1));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  // TF 由 odometryCallback 直接驱动, 不再用独立 wall timer。角度源只有约 20Hz,
  // 原先 50Hz 定时重发不带来新信息, 只制造 62% 的重复帧 (实测) —— 重复帧让 tf2
  // 无法在真实台阶之间插值, 反而把整个 odom 台阶挤进单个 20ms 桶。
}

void FakeVelTransform::cmdSpinCallback(const example_interfaces::msg::Float32::SharedPtr msg)
{
  spin_speed_ = msg->data;
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // 角度与它的真实时刻一同缓存。/odometry 从传感器时刻到本回调实测滞后约 206ms,
  // 因此"现在收到"绝不等于"现在的姿态"; 用 now() 给 TF 盖章会让 tf2 认为这是当前
  // 姿态, 而 gimbal_yaw_fake 正是 Nav2 controller 的规划系 —— 谎报的时刻会让规划系
  // 在惯性系里自己抖 (实测朝向误差 RMS 14.07°, 峰峰 94°, 逐帧增量 2.52°)。
  // 用真实 stamp 盖章后, tf2 能正确插值, 上述误差降到 1.82° / 13° / 0.77°。
  double angle = tf2::getYaw(msg->pose.pose.orientation);
  {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    current_robot_base_angle_ = angle;
  }
  publishTransform(angle, rclcpp::Time(msg->header.stamp));
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double angle;
  {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    angle = current_robot_base_angle_;
  }
  auto aft_tf_vel = transformVelocity(msg->twist, angle);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishTransform(double angle, const rclcpp::Time & stamp)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = robot_base_frame_;
  t.child_frame_id = fake_robot_base_frame_;
  tf2::Quaternion q;
  q.setRPY(0, 0, -angle);
  t.transform.rotation = tf2::toMsg(q);
  tf_broadcaster_->sendTransform(t);
}

geometry_msgs::msg::Twist FakeVelTransform::transformVelocity(
  const geometry_msgs::msg::Twist & twist, float yaw_diff)
{
  geometry_msgs::msg::Twist aft_tf_vel;
  aft_tf_vel.angular.z = twist.angular.z + spin_speed_;
  aft_tf_vel.linear.x = twist.linear.x * cos(yaw_diff) + twist.linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist.linear.x * sin(yaw_diff) + twist.linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
