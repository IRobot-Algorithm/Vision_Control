#pragma once

#include <memory>
#include <string>
#include <functional>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <skider_interface/msg/imu.hpp>

#include "mahony_ahrs.hpp"

using namespace std::chrono_literals;

class ImuSensorNode {
 public:
  explicit ImuSensorNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() {
    return imu_sensor_node_->get_node_base_interface();
  }

 private:
  void imu_raw_msg_callback(const sensor_msgs::msg::Imu& msg) const;
  void loop_1000Hz();

 private:
  rclcpp::Node::SharedPtr imu_sensor_node_;
  rclcpp::TimerBase::SharedPtr timer_1000Hz_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_raw_subscription_;
  rclcpp::Publisher<skider_interface::msg::Imu>::SharedPtr imu_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> imu_tf_broadcaster_;
  std::shared_ptr<sensor_sdk::imu::AHRSInterface> imu_ahrs_processer_;

 private:
  // params
  std::string imu_raw_sbuscribe_topic_name_;
  std::string imu_data_public_topic_name_;
  std::string processer_algorithm_;
  std::string tf_root_;
  std::string tf_branch_;
  bool publish_tf_;
  double mahony_adjust_kp_;
  double mahony_adjust_ki_;
  int mahony_frequency_;
  std::vector<double> imu_externel_parmeters_z_;
  std::vector<double> imu_externel_parmeters_y_;
  std::vector<double> imu_externel_parmeters_x_;
};
