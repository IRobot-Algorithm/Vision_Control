/*
 * @Description: 
 * @Version: 1.0
 * @Autor: Qylann
 * @Date: 2022-03-29 20:37:02
 * @LastEditors: Qylann
 * @LastEditTime: 2022-04-05 15:42:37
 */


#include "remote_sensor_node.hpp"

RemoteSensorNode::RemoteSensorNode(const rclcpp::NodeOptions & options)
{
    
    remote_sensor_node_ = std::make_shared<rclcpp::Node>("remote_sensor_node", options);
    RCLCPP_INFO(remote_sensor_node_->get_logger(), "Remote Node Begin");

    // ****** get parameters ******
    
    std::map<std::string, std::string> string_params {
        {"joy_sbus_sbuscribe_topic_name", ""},
        {"joy_data_publish_topic_name", ""},
    };
    remote_sensor_node_->declare_parameters("", string_params);

    remote_sensor_node_->get_parameter<std::string>("joy_sbus_sbuscribe_topic_name", joy_sbus_sbuscribe_topic_name_);
    remote_sensor_node_->get_parameter<std::string>("joy_data_publish_topic_name", joy_data_publish_topic_name_);

    
    remote_ = std::make_shared<sensor_sdk::remote::SbusRemote>();


    RCLCPP_INFO(remote_sensor_node_->get_logger(), "Sbuscribe sbus");
    sbus_subscription_ = remote_sensor_node_->create_subscription<skider_interface::msg::Sbus>(
        joy_sbus_sbuscribe_topic_name_, 10, std::bind(&RemoteSensorNode::sbus_msg_callback, this, std::placeholders::_1));
    

    RCLCPP_INFO(remote_sensor_node_->get_logger(), "Init joy Publisher");
    joy_publisher_ = remote_sensor_node_->create_publisher<sensor_msgs::msg::Joy>(
        joy_data_publish_topic_name_, 10);


    RCLCPP_INFO(remote_sensor_node_->get_logger(), "Init Timer 1000Hz");
    // although the duration is 0.5ms, the frequency of /skider/joy/data callback is 1khz. it is weird
    timer_1000Hz_ = remote_sensor_node_->create_wall_timer(0.5ms, std::bind(&RemoteSensorNode::loop_1000Hz, this));
    // timer_1000Hz_ = remote_sensor_node_->create_wall_timer(1ms, std::bind(&RemoteSensorNode::loop_1000Hz, this));
}

void RemoteSensorNode::loop_1000Hz()
{
    remote_->process();

    sensor_msgs::msg::Joy joy_msg;
    joy_msg.header.set__frame_id("sbus");
    joy_msg.header.set__stamp(remote_sensor_node_->get_clock()->now());
    for (uint i = 0; i < remote_->axes_size; i++) {
        joy_msg.axes.push_back(remote_->axes[i]);
    }
    for (uint i = 0; i < remote_->buttons_size; i++) {
        joy_msg.buttons.push_back(remote_->buttons[i]);
    }
    
    joy_publisher_->publish(joy_msg);

}

void RemoteSensorNode::sbus_msg_callback(const skider_interface::msg::Sbus & msg) const
{
    unsigned char sbus_buffer[18];
    for (uint i = 0; i < 18; i++) {
        sbus_buffer[i] = msg.ch[i];
    }
    remote_->update(sbus_buffer);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto remote_sensor_node = std::make_shared<RemoteSensorNode>();
    rclcpp::spin(remote_sensor_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}