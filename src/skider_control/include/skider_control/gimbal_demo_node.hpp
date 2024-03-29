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
#include <geometry_msgs/msg/vector3.hpp>

#include <skider_interface/msg/gimbal_command.hpp>
#include <skider_interface/msg/gimbal_debug.hpp>
#include <skider_interface/msg/imu.hpp>
#include <skider_interface/msg/gimbal_state.hpp>
// #include <../../skider_utils/include/utils.hpp>
#include <skider_utils/utils.hpp>
#include <skider_utils/pid.hpp>

using namespace std::chrono_literals;

class GimbalControlerDemoNode : public rclcpp::Node {
 public:
  explicit GimbalControlerDemoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  void joy_msg_callback(const sensor_msgs::msg::Joy& msg);
  void imu_msg_callback(const skider_interface::msg::Imu& msg);
  void gimbal_msg_callback(const skider_interface::msg::GimbalState& msg);
  void autoaim_msg_callback(const geometry_msgs::msg::Vector3& msg);
  void loop_calculate();

 private:
  rclcpp::Subscription<skider_interface::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;
  rclcpp::Subscription<skider_interface::msg::GimbalState>::SharedPtr gimbal_state_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr autoaim_target_subscription_;  // autoaim

  rclcpp::Publisher<skider_interface::msg::GimbalCommand>::SharedPtr gimbal_command_publisher_;
  rclcpp::Publisher<skider_interface::msg::GimbalDebug>::SharedPtr gimbal_debug_publisher_;
  skider_interface::msg::GimbalDebug gimbal_debug_msg_;

  rclcpp::CallbackGroup::SharedPtr calculate_call_backgroup_;
  rclcpp::TimerBase::SharedPtr follow_init_timer_;
  // rclcpp::TimerBase::SharedPtr calculate_pid_timer_;

 private:
  // imu
  double imu_yaw_;
  double imu_pitch_;
  double imu_roll_;
  double w_pitch_;
  double w_yaw_;

  // remote
  double yaw_angle_set_;
  double pitch_angle_set_;

  // autoaim
  double autoaim_pitch_, autoaim_yaw_, autoaim_roll_;

  // PID
  PID pid_yaw_remote_in_, pid_yaw_remote_out_;
  PID pid_yaw_init_in_, pid_yaw_init_out_;
  PID pid_yaw_vision_in_, pid_yaw_vision_out_;
  PID pid_pitch_remote_in_, pid_pitch_remote_out_;
  PID pid_pitch_vision_in_, pid_pitch_vision_out_;
  PID pid_ammor_, pid_ammol_, pid_rotor_;

  std::vector<double> pid_yaw_remote_in_params_, pid_yaw_remote_out_params_;
  std::vector<double> pid_yaw_init_in_params_, pid_yaw_init_out_params_;
  std::vector<double> pid_yaw_vision_in_params_, pid_yaw_vision_out_params_;
  std::vector<double> pid_pitch_remote_in_params_, pid_pitch_remote_out_params_;
  std::vector<double> pid_pitch_vision_in_params_, pid_pitch_vision_out_params_;
  std::vector<double> pid_ammor_params_, pid_ammol_params_, pid_rotor_params_;

  // gimbal state feedback
  double yaw_angle_, pitch_angle_;
  double ammor_speed_, ammol_speed_, rotor_speed_;

  // init & goal
  bool follow_init_{false};
  bool ammo_enable_{false};
  bool rotor_enable_{false};
  double yaw_zero_angle_;
  double ammo_goal_speed_, rotor_goal_speed_;

  // ---joy command---
  RobotState robot_state_{RobotState::ChassisWeakGimbalWeak};
  bool shoot_request_{false};
  bool spin_request_{false};

 public:
  skider_interface::msg::GimbalCommand gimbal_command_msg_;
};
