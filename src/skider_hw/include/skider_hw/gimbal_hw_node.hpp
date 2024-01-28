#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <skider_interface/msg/sbus.hpp>
#include <skider_interface/msg/gimbal_command.hpp>
#include <skider_interface/msg/chassis_state.hpp>
#include <skider_interface/msg/chassis_command.hpp>
#include <skider_interface/msg/gimbal_state.hpp>
#include <skider_interface/msg/device_online.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include "usbcdc_transporter.hpp"
#include "transport_package.h"
#include "can.hpp"

using namespace std::chrono_literals;

class GimbalHWNode
{
public:
    explicit GimbalHWNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
    {
        return gimbal_hw_node_->get_node_base_interface();
    }

private:
    rclcpp::Node::SharedPtr gimbal_hw_node_;

    void loop_receive();
    void loop_send();
    void loop_device_online();

    void gimbal_command_msg_callback(const skider_interface::msg::GimbalCommand &msg);
    void chassis_command_msg_callback(const skider_interface::msg::ChassisCommand &msg);
    void gimbalRecevieCallBack();
    void chassisRecevieCallBack();

    std::thread gimbal_recevie_thread_;
    std::thread chassis_recevie_thread_;

    rclcpp::CallbackGroup::SharedPtr send_call_backgroup_;
    rclcpp::CallbackGroup::SharedPtr receive_call_backgroup_;

    // ---timer---
    rclcpp::TimerBase::SharedPtr timer_10000Hz_;
    rclcpp::TimerBase::SharedPtr timer_1000Hz_;
    rclcpp::TimerBase::SharedPtr device_online_timer_;

    // ---publisher---
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_raw_publisher_;
    rclcpp::Publisher<skider_interface::msg::Sbus>::SharedPtr sbus_publisher_;
    rclcpp::Publisher<skider_interface::msg::ChassisState>::SharedPtr chassis_state_publisher_;
    rclcpp::Publisher<skider_interface::msg::GimbalState>::SharedPtr gimbal_state_publisher_;
    rclcpp::Publisher<skider_interface::msg::DeviceOnline>::SharedPtr device_online_publisher_;

    // ---subscription---
    rclcpp::Subscription<skider_interface::msg::GimbalCommand>::SharedPtr gimbal_command_subscription_;
    rclcpp::Subscription<skider_interface::msg::ChassisCommand>::SharedPtr chassis_command_subscription_;

    // ---transporter---
    std::shared_ptr<transporter_sdk::TransporterInterface> transporter_;

    // ---debug---
    std_msgs::msg::Header stamp_;

    // ---params---
    std::string imu_raw_publish_topic_name_;
    std::string sbus_publish_topic_name_;
    int gimbal_interface_usb_vid_;
    int gimbal_interface_usb_pid_;
    int gimbal_interface_usb_read_endpoint_;
    int gimbal_interface_usb_write_endpoint_;
    int gimbal_interface_usb_read_timeout_;
    int gimbal_interface_usb_write_timeout_;

    // ---can---
    transporter_sdk::Can can0_{transporter_sdk::Can(0)};
    transporter_sdk::Can can1_{transporter_sdk::Can(1)};

    // ---buffer---
    u_char buf_gimbal_[8];
    u_char buf_shooter_[8];
    u_char buf_chassis_[8];

public:
    // ---msg---
    skider_interface::msg::GimbalState gimbal_state_msg_;
    skider_interface::msg::ChassisState chassis_state_msg_;
    skider_interface::msg::DeviceOnline device_online_msg_;
};
