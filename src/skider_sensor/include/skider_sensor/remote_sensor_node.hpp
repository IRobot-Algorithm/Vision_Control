/*
 * @Description:
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-04-03 22:19:32
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 12:38:37
 */

#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joy.hpp>
#include <skider_interface/msg/sbus.hpp>

#include "sbus_remote.hpp"

using namespace std::chrono_literals;

class RemoteSensorNode {
 public:
  explicit RemoteSensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return remote_sensor_node_->get_node_base_interface();
  }

 private:
  void sbus_msg_callback(const skider_interface::msg::Sbus& msg) const;
  void loop_1000Hz();

 private:
  rclcpp::Node::SharedPtr remote_sensor_node_;
  rclcpp::TimerBase::SharedPtr timer_1000Hz_;
  rclcpp::Subscription<skider_interface::msg::Sbus>::SharedPtr sbus_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  std::shared_ptr<sensor_sdk::remote::RemoteBase> remote_;

 private:
  // params
  std::string joy_sbus_sbuscribe_topic_name_;
  std::string joy_data_publish_topic_name_;
};
