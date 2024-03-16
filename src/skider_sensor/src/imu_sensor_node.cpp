
#include "imu_sensor_node.hpp"

#include <tf2/LinearMath/Matrix3x3.h>

ImuSensorNode::ImuSensorNode(const rclcpp::NodeOptions& options) {
  imu_sensor_node_ = std::make_shared<rclcpp::Node>("imu_sensor_node", options);
  RCLCPP_INFO(imu_sensor_node_->get_logger(), "Node Begin");

  // ****** get parameters ******

  std::map<std::string, std::string> string_params{
      {"imu_raw_sbuscribe_topic_name", ""},
      {"imu_data_public_topic_name", ""},
      {"processer_algorithm", ""},
      {"tf_root", ""},
      {"tf_branch", ""},
  };
  imu_sensor_node_->declare_parameters("", string_params);
  std::map<std::string, bool> bool_params{
      {"publish_tf", true},
  };
  imu_sensor_node_->declare_parameters("", bool_params);
  std::map<std::string, double> double_params{
      {"mahony_adjust_kp", 1.0},
      {"mahony_adjust_ki", 0.0},
  };
  imu_sensor_node_->declare_parameters("", double_params);
  std::map<std::string, int> int_params{
      {"mahony_frequency", 1000},
  };
  imu_sensor_node_->declare_parameters("", int_params);
  std::map<std::string, std::vector<double>> vector3d_params{
      {"imu_externel_parmeters_z", {0.0, 0.0, 0, 0}},
      {"imu_externel_parmeters_y", {0.0, 0.0, 0.0}},
      {"imu_externel_parmeters_x", {0.0, 0.0, 0.0}},
  };
  imu_sensor_node_->declare_parameters("", vector3d_params);

  imu_sensor_node_->get_parameter<std::string>("imu_raw_sbuscribe_topic_name", imu_raw_sbuscribe_topic_name_);
  imu_sensor_node_->get_parameter<std::string>("imu_data_public_topic_name", imu_data_public_topic_name_);
  imu_sensor_node_->get_parameter<std::string>("processer_algorithm", processer_algorithm_);
  imu_sensor_node_->get_parameter<std::string>("tf_root", tf_root_);
  imu_sensor_node_->get_parameter<std::string>("tf_branch", tf_branch_);
  imu_sensor_node_->get_parameter<bool>("publish_tf", publish_tf_);
  imu_sensor_node_->get_parameter<double>("mahony_adjust_kp", mahony_adjust_kp_);
  imu_sensor_node_->get_parameter<double>("mahony_adjust_ki", mahony_adjust_ki_);
  imu_sensor_node_->get_parameter<int>("mahony_frequency", mahony_frequency_);
  imu_sensor_node_->get_parameter<std::vector<double>>("imu_externel_parmeters_z", imu_externel_parmeters_z_);
  imu_sensor_node_->get_parameter<std::vector<double>>("imu_externel_parmeters_y", imu_externel_parmeters_y_);
  imu_sensor_node_->get_parameter<std::vector<double>>("imu_externel_parmeters_x", imu_externel_parmeters_x_);
  RCLCPP_INFO(imu_sensor_node_->get_logger(), "%f, %f, %f", imu_externel_parmeters_z_[0], imu_externel_parmeters_z_[1], imu_externel_parmeters_z_[2]);
  RCLCPP_INFO(imu_sensor_node_->get_logger(), "%f, %f, %f", imu_externel_parmeters_y_[0], imu_externel_parmeters_y_[1], imu_externel_parmeters_y_[2]);
  RCLCPP_INFO(imu_sensor_node_->get_logger(), "%f, %f, %f", imu_externel_parmeters_x_[0], imu_externel_parmeters_x_[1], imu_externel_parmeters_x_[2]);

  if (processer_algorithm_ == "Mahony") {
    RCLCPP_INFO(imu_sensor_node_->get_logger(), "Init IMU Processer");
    imu_ahrs_processer_ = std::make_shared<sensor_sdk::imu::MahonyAHRS>(mahony_adjust_kp_, mahony_adjust_ki_, mahony_frequency_);
  }

  RCLCPP_INFO(imu_sensor_node_->get_logger(), "Subscribe IMU raw data");
  imu_raw_subscription_ = imu_sensor_node_->create_subscription<sensor_msgs::msg::Imu>(
      imu_raw_sbuscribe_topic_name_, 10, std::bind(&ImuSensorNode::imu_raw_msg_callback, this, std::placeholders::_1));

  RCLCPP_INFO(imu_sensor_node_->get_logger(), "Init IMU data Publisher");
  imu_publisher_ = imu_sensor_node_->create_publisher<skider_interface::msg::Imu>(
      imu_data_public_topic_name_, 10);

  if (publish_tf_ == true) {
    RCLCPP_INFO(imu_sensor_node_->get_logger(), "Init IMU tf Broadcaster");
    imu_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*imu_sensor_node_);
  }

  RCLCPP_INFO(imu_sensor_node_->get_logger(), "Init Timer 1000Hz");
  timer_1000Hz_ = imu_sensor_node_->create_wall_timer(1ms, std::bind(&ImuSensorNode::loop_1000Hz, this));
}

void ImuSensorNode::loop_1000Hz() {
  // update quarternion
  imu_ahrs_processer_->calculate();

  // publish imu
  sensor_msgs::msg::Imu imu_data;
  // imu_data.header.set__stamp(imu_sensor_node_->get_clock()->now());
  // imu_data.header.set__frame_id("imu_data");
  imu_data.orientation.set__w(imu_ahrs_processer_->Quat_.w());
  imu_data.orientation.set__x(imu_ahrs_processer_->Quat_.x());
  imu_data.orientation.set__y(imu_ahrs_processer_->Quat_.y());
  imu_data.orientation.set__z(imu_ahrs_processer_->Quat_.z());
  imu_data.angular_velocity.set__x(imu_ahrs_processer_->imu_raw_.gyro_x);
  imu_data.angular_velocity.set__y(imu_ahrs_processer_->imu_raw_.gyro_y);
  imu_data.angular_velocity.set__z(imu_ahrs_processer_->imu_raw_.gyro_z);
  imu_data.linear_acceleration.set__x(imu_ahrs_processer_->imu_raw_.accl_x);
  imu_data.linear_acceleration.set__y(imu_ahrs_processer_->imu_raw_.accl_y);
  imu_data.linear_acceleration.set__z(imu_ahrs_processer_->imu_raw_.accl_z);

  skider_interface::msg::Imu imu_msg;
  imu_msg.header.stamp = imu_sensor_node_->get_clock()->now();
  imu_msg.header.frame_id = "imu_data";
  imu_msg.imu = imu_data;

  tf2::Matrix3x3 mat(tf2::Quaternion(imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w));
  mat.getEulerYPR(imu_msg.imu_yaw, imu_msg.imu_pitch, imu_msg.imu_roll);

  imu_publisher_->publish(imu_msg);

  // broadcast tf
  if (publish_tf_ == true) {
    geometry_msgs::msg::TransformStamped imu_tf;
    imu_tf.header.set__stamp(imu_sensor_node_->get_clock()->now());
    imu_tf.header.frame_id = tf_root_;
    imu_tf.child_frame_id = tf_branch_;
    imu_tf.transform.translation.set__x(0);
    imu_tf.transform.translation.set__y(0);
    imu_tf.transform.translation.set__z(0);
    imu_tf.transform.rotation.set__w(imu_ahrs_processer_->Quat_.w());
    imu_tf.transform.rotation.set__x(imu_ahrs_processer_->Quat_.x());
    imu_tf.transform.rotation.set__y(imu_ahrs_processer_->Quat_.y());
    imu_tf.transform.rotation.set__z(imu_ahrs_processer_->Quat_.z());
    imu_tf_broadcaster_->sendTransform(imu_tf);
  }

  // RCLCPP_INFO(imu_sensor_node_->get_logger(), "imu_tf_broadcaster=>>");
}

void ImuSensorNode::imu_raw_msg_callback(const sensor_msgs::msg::Imu& msg) const {
  // RCLCPP_INFO(imu_sensor_node_->get_logger(), "receive imu raw data!!");
  sensor_sdk::imu::ImuRawSixAxis imu_raw;
  imu_raw.gyro_z = imu_externel_parmeters_z_[0] * msg.angular_velocity.z + imu_externel_parmeters_z_[1] * msg.angular_velocity.y + imu_externel_parmeters_z_[2] * msg.angular_velocity.x;
  imu_raw.gyro_y = imu_externel_parmeters_y_[0] * msg.angular_velocity.z + imu_externel_parmeters_y_[1] * msg.angular_velocity.y + imu_externel_parmeters_y_[2] * msg.angular_velocity.x;
  imu_raw.gyro_x = imu_externel_parmeters_x_[0] * msg.angular_velocity.z + imu_externel_parmeters_x_[1] * msg.angular_velocity.y + imu_externel_parmeters_x_[2] * msg.angular_velocity.x;
  imu_raw.accl_z = imu_externel_parmeters_z_[0] * msg.linear_acceleration.z + imu_externel_parmeters_z_[1] * msg.linear_acceleration.y + imu_externel_parmeters_z_[2] * msg.linear_acceleration.x;
  imu_raw.accl_y = imu_externel_parmeters_y_[0] * msg.linear_acceleration.z + imu_externel_parmeters_y_[1] * msg.linear_acceleration.y + imu_externel_parmeters_y_[2] * msg.linear_acceleration.x;
  imu_raw.accl_x = imu_externel_parmeters_x_[0] * msg.linear_acceleration.z + imu_externel_parmeters_x_[1] * msg.linear_acceleration.y + imu_externel_parmeters_x_[2] * msg.linear_acceleration.x;
  imu_ahrs_processer_->update(imu_raw);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto imu_sensor_node = std::make_shared<ImuSensorNode>();
  rclcpp::spin(imu_sensor_node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
