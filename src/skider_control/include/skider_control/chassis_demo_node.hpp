#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <iostream>

#include <Eigen/Eigen>
#include <tf2/LinearMath/Matrix3x3.h>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <skider_interface/msg/chassis_command.hpp>
// #include <skider_interface/msg/debug.hpp>
#include <skider_interface/msg/imu.hpp>
#include <skider_interface/msg/chassis_state.hpp>
#include <skider_interface/msg/gimbal_state.hpp>
#include <skider_interface/msg/gimbal_command.hpp>
#include <skider_utils/pid.hpp>

using namespace std::chrono_literals;

class ChassisControlerDemoNode : public rclcpp::Node {
 public:
  explicit ChassisControlerDemoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void loop_10000Hz();
  void joy_msg_callback(const sensor_msgs::msg::Joy& msg);
  void imu_msg_callback(const skider_interface::msg::Imu& msg);
  void chassis_msg_callback(const skider_interface::msg::ChassisState& msg);
  void gimbal_msg_callback(const skider_interface::msg::GimbalState& msg);
  void gimbal_command_msg_callback(const skider_interface::msg::GimbalCommand& msg);

 private:
  rclcpp::TimerBase::SharedPtr timer_1000Hz_;
  rclcpp::Subscription<skider_interface::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<skider_interface::msg::ChassisState>::SharedPtr chassis_state_subscription_;
  rclcpp::Subscription<skider_interface::msg::GimbalState>::SharedPtr gimbal_state_subscription_;
  rclcpp::Subscription<skider_interface::msg::GimbalCommand>::SharedPtr gimbal_command_subscription_;

  rclcpp::Publisher<skider_interface::msg::ChassisCommand>::SharedPtr chassis_command_publisher_;
  // rclcpp::Publisher<skider_interface::msg::Debug>::SharedPtr debug_publisher_;
  // skider_interface::msg::Debug debug_msg_;

 private:
  // callback
  double imu_yaw_;
  double vx_set_, vy_set_;
  bool button1_, button2_;
  double axes4_;

  double vx_solve_, vy_solve_;
  double chassis_speed_[4] = {0};
  std_msgs::msg::Header stamp_;

  // params
  double spin_w_;
  double yaw_zero_angle_;
  // double yaw_zero_angle_ = 7792;
  PID pid_follow_;
  std::vector<double> pid_follow_params_;
  std::vector<PID> pid_vec_;
  std::vector<double> pid1_params_, pid2_params_, pid3_params_, pid4_params_;
  // 2-----battary-----1
  //|                 |
  //|                 |
  //|                 |
  //|                 |
  //|                 |
  //|                 |
  //|                 |
  // 3-----------------4

 public:
  int16_t chassis_state_[4] = {0};
  double follow_w_;

  // gimbal state feedback
  double yaw_angle_, pitch_angle_;
  double ammor_speed_, ammol_speed_, rotor_speed_;

  // gimbal command
  bool follow_init_;
};
