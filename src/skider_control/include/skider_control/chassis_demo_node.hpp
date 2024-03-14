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

using namespace std::chrono_literals;

class Limit {
 public:
  Limit() {
  }
  Limit(double min, double max) {
    max_ = max;
    min_ = min;
    out_ = 0;
  }

  double get(double in) {
    if (out_ + in < min_) {
      out_ = min_;
    } else if (out_ + in > max_) {
      out_ = max_;
    } else {
      out_ += in;
    }
    return out_;
  }

 private:
  double max_, min_;
  double out_;
};

class PID {
 public:
  PID() {
  }
  PID(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    err_ = 0;
    err_last_ = 0;
    command_ = 0;
  }
  double calculate(double in, double state_now) {
    err_ = in - state_now;
    // std::cout<<"err_: "<<err_<<std::endl;
    double i_sum_limit = i_sum_limit_.get(err_ * ki_);
    // std::cout<<"i_sum_limit: "<<i_sum_limit<<std::endl;
    command_ = kp_ * err_ + i_sum_limit + kd_ * (err_ - err_last_);

    err_last_ = err_;

    return command_;
  }
  double calculate_robust(double in, double state_now) {
    err_ = in - state_now;
    // std::cout<<"err_: "<<err_<<std::endl;
    double i_sum_limit = i_sum_limit_.get(err_ * ki_);
    // std::cout<<"i_sum_limit: "<<i_sum_limit<<std::endl;
    command_ = kp_ * err_ + i_sum_limit + kd_ * (err_ - err_last_);
    if (err_ > 100) command_ += 200;
    if (err_ < -100) command_ -= 200;
    err_last_ = err_;

    return command_;
  }
  Limit i_sum_limit_;

 private:
  double kp_, ki_, kd_;
  double err_, err_last_;
  double command_;
};

class ChassisControlerDemoNode {
 public:
  explicit ChassisControlerDemoNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return chassis_controler_demo_node_->get_node_base_interface();
  }

 private:
  void loop_10000Hz();
  void joy_msg_callback(const sensor_msgs::msg::Joy& msg);
  void imu_msg_callback(const skider_interface::msg::Imu& msg);
  void chassis_msg_callback(const skider_interface::msg::ChassisState& msg);
  void gimbal_msg_callback(const skider_interface::msg::GimbalState& msg);
  void gimbal_command_msg_callback(const skider_interface::msg::GimbalCommand& msg);

 private:
  rclcpp::TimerBase::SharedPtr timer_1000Hz_;
  rclcpp::Node::SharedPtr chassis_controler_demo_node_;
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
