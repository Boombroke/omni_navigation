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

  this->declare_parameter<std::string>("odom_frame", "odom");
  this->declare_parameter<std::string>("robot_base_frame", "gimbal_yaw");
  this->declare_parameter<std::string>("fake_robot_base_frame", "gimbal_yaw_fake");
  this->declare_parameter<std::string>("odom_topic", "odom");
  this->declare_parameter<std::string>("input_cmd_vel_topic", "");
  this->declare_parameter<std::string>("output_cmd_vel_topic", "");

  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("robot_base_frame", robot_base_frame_);
  this->get_parameter("fake_robot_base_frame", fake_robot_base_frame_);
  this->get_parameter("odom_topic", odom_topic_);
  this->get_parameter("input_cmd_vel_topic", input_cmd_vel_topic_);
  this->get_parameter("output_cmd_vel_topic", output_cmd_vel_topic_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  cmd_vel_chassis_pub_ =
    this->create_publisher<geometry_msgs::msg::Twist>(output_cmd_vel_topic_, 1);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    input_cmd_vel_topic_, 10,
    std::bind(&FakeVelTransform::cmdVelCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic_, rclcpp::SensorDataQoS(),
    std::bind(&FakeVelTransform::odometryCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20), std::bind(&FakeVelTransform::publishTransform, this));
}

void FakeVelTransform::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg)
{
  // 仅作 TF 查询失败时的回退角源。正常路径走 getRobotBaseAngle() 的高频 TF 插值。
  std::lock_guard<std::mutex> lock(angle_mutex_);
  current_robot_base_angle_ = tf2::getYaw(msg->pose.pose.orientation);
}

double FakeVelTransform::getRobotBaseAngle(const rclcpp::Time & stamp)
{
  // 底盘持续自旋时,补偿角必须取自高频角源:odom→gimbal_yaw 链路里自旋在
  // chassis→gimbal_yaw(~200Hz)一段,tf2 lookup 天然按该高频 TF 插值,绕开
  // 旧实现用 13Hz odom 采样导致的欠采样(绕圈根因)。
  // 全部用【非阻塞】lookup(无 timeout):否则在 30Hz 回调里阻塞等待 TF 会拖垮节点。
  // 级联:先对齐 cmd 时刻 stamp → 退到最新可用(自旋分量仍 ~200Hz 新)→ 退到冻结 odom 角。
  try {
    auto tf = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, stamp);
    return tf2::getYaw(tf.transform.rotation);
  } catch (const tf2::TransformException &) {
  }
  try {
    auto tf = tf_buffer_->lookupTransform(odom_frame_, robot_base_frame_, tf2::TimePointZero);
    return tf2::getYaw(tf.transform.rotation);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "lookupTransform %s->%s 失败 (%s); 回退最近 odom 角",
      odom_frame_.c_str(), robot_base_frame_.c_str(), ex.what());
    std::lock_guard<std::mutex> lock(angle_mutex_);
    return current_robot_base_angle_;
  }
}

void FakeVelTransform::cmdVelCallback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  double angle = getRobotBaseAngle(msg->header.stamp);
  auto aft_tf_vel = transformVelocity(msg->twist, angle);
  cmd_vel_chassis_pub_->publish(aft_tf_vel);
}

void FakeVelTransform::publishTransform()
{
  // 用最新可用的高频角(而非冻结的 13Hz odom 值)发 gimbal_yaw→gimbal_yaw_fake,
  // 使反自旋虚拟系在两次 odom 更新之间仍保持惯性对齐。
  // stamp 保持 now():该 TF 定义"当前"的 fake 系,下游按当前时刻查询,避免过去戳导致外推失败。
  double angle;
  try {
    auto tf = tf_buffer_->lookupTransform(
      odom_frame_, robot_base_frame_, tf2::TimePointZero);
    angle = tf2::getYaw(tf.transform.rotation);
  } catch (const tf2::TransformException &) {
    std::lock_guard<std::mutex> lock(angle_mutex_);
    angle = current_robot_base_angle_;
  }
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
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
  aft_tf_vel.angular.z = twist.angular.z;
  aft_tf_vel.linear.x = twist.linear.x * cos(yaw_diff) + twist.linear.y * sin(yaw_diff);
  aft_tf_vel.linear.y = -twist.linear.x * sin(yaw_diff) + twist.linear.y * cos(yaw_diff);
  return aft_tf_vel;
}

}  // namespace fake_vel_transform

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fake_vel_transform::FakeVelTransform)
