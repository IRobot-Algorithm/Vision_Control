#include "gimbal_hw_node.hpp"

GimbalHWNode::GimbalHWNode(const rclcpp::NodeOptions &options)
{
    gimbal_hw_node_ = std::make_shared<rclcpp::Node>("gimbal_hw_node", options);
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "GimbalHWNode Begin");

    // ****** get parameters ******

    std::map<std::string, std::string> topic_name_params{
        {"imu_raw_publish_topic_name", ""},
        {"sbus_publish_topic_name", ""},
    };

    std::map<std::string, int> transporter_params{
        {"gimbal_interface_usb_vid", 0},
        {"gimbal_interface_usb_pid", 0},
        {"gimbal_interface_usb_read_endpoint", 0},
        {"gimbal_interface_usb_write_endpoint", 0},
        {"gimbal_interface_usb_read_timeout", 0},
        {"gimbal_interface_usb_write_timeout", 0},
    };

    gimbal_hw_node_->declare_parameters("", topic_name_params);
    // topic name
    gimbal_hw_node_->get_parameter<std::string>("imu_raw_publish_topic_name", imu_raw_publish_topic_name_);
    gimbal_hw_node_->get_parameter<std::string>("sbus_publish_topic_name", sbus_publish_topic_name_);

    gimbal_hw_node_->declare_parameters("", transporter_params);
    // transporter parameter
    std::cout << "gimbal_interface_usb_vid_: " << gimbal_interface_usb_vid_ << std::endl;

    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_vid", gimbal_interface_usb_vid_);
    std::cout << "gimbal_interface_usb_vid_: " << gimbal_interface_usb_vid_ << std::endl;

    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_pid", gimbal_interface_usb_pid_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_read_endpoint", gimbal_interface_usb_read_endpoint_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_write_endpoint", gimbal_interface_usb_write_endpoint_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_read_timeout", gimbal_interface_usb_read_timeout_);
    gimbal_hw_node_->get_parameter<int>("gimbal_interface_usb_write_timeout", gimbal_interface_usb_write_timeout_);

    send_call_backgroup_ = gimbal_hw_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    receive_call_backgroup_ = gimbal_hw_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // ---publisher---
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init imu_raw Publisher");
    imu_raw_publisher_ = gimbal_hw_node_->create_publisher<sensor_msgs::msg::Imu>(
        imu_raw_publish_topic_name_, 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init sbus Publisher");
    sbus_publisher_ = gimbal_hw_node_->create_publisher<skider_interface::msg::Sbus>(
        sbus_publish_topic_name_, 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init chassis state Publisher");
    chassis_state_publisher_ = gimbal_hw_node_->create_publisher<skider_interface::msg::ChassisState>(
        "/skider/state/chassis", 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init gimbal state Publisher");
    gimbal_state_publisher_ = gimbal_hw_node_->create_publisher<skider_interface::msg::GimbalState>(
        "/skider/state/gimbal", 10);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init device online Publisher");
    device_online_publisher_ = gimbal_hw_node_->create_publisher<skider_interface::msg::DeviceOnline>(
        "/skider/device_online", 10);

    // ---subscription---
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Subscribe Gimbal Command");
    gimbal_command_subscription_ = gimbal_hw_node_->create_subscription<skider_interface::msg::GimbalCommand>(
        "/skider/command/gimbal", 10, std::bind(&GimbalHWNode::gimbal_command_msg_callback, this, std::placeholders::_1));

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Subscribe Chassis Command");
    chassis_command_subscription_ = gimbal_hw_node_->create_subscription<skider_interface::msg::ChassisCommand>(
        "/skider/command/chassis", 10, std::bind(&GimbalHWNode::chassis_command_msg_callback, this, std::placeholders::_1));

    // ---timer---
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Timer ");

    // the max actual frequency is 1000HZ. Therefore use duration less than 1000us is meaningless.
    timer_10000Hz_ = gimbal_hw_node_->create_wall_timer(1000us, std::bind(&GimbalHWNode::loop_receive, this), receive_call_backgroup_);
    timer_1000Hz_ = gimbal_hw_node_->create_wall_timer(1000us, std::bind(&GimbalHWNode::loop_send, this), send_call_backgroup_);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Timer ");
    device_online_timer_ = gimbal_hw_node_->create_wall_timer(500ms, std::bind(&GimbalHWNode::loop_device_online, this), send_call_backgroup_);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Recevie Thread ");
    this->recevie_thread_ = std::thread(&GimbalHWNode::recevieCallBack, this);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Init Transporter");
    transporter_ = std::make_shared<transporter_sdk::UsbcdcTransporter>(
        gimbal_interface_usb_vid_,
        gimbal_interface_usb_pid_,
        gimbal_interface_usb_read_endpoint_,
        gimbal_interface_usb_write_endpoint_,
        gimbal_interface_usb_read_timeout_,
        gimbal_interface_usb_write_timeout_);

    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Open Transporter");
    if (transporter_->open() == true)
    {
        RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Success");
    }
    else
    {
        RCLCPP_INFO(gimbal_hw_node_->get_logger(), "FAILED!!!");
    }
    RCLCPP_INFO(gimbal_hw_node_->get_logger(), "Finish Init");
}

void GimbalHWNode::loop_receive()
{
    // std::cout << "start: " << gimbal_hw_node_->get_clock()->now().nanoseconds() << std::endl;

    //---usb---
    transport_package::GimbalHWReceivePackage package;
    int read_size = transporter_->read((unsigned char *)&package, 32);
    // RCLCPP_INFO(gimbal_hw_node_->get_logger(), "read size : %d", read_size);
    if (read_size == 32)
    {

        device_online_msg_.sensor = true;

        sensor_msgs::msg::Imu imu_raw;

        imu_raw.header.set__frame_id("imu_raw");
        imu_raw.header.set__stamp(gimbal_hw_node_->get_clock()->now());

        imu_raw.angular_velocity.set__x(package.imu.gyro_x / 32768.0 * 2000.0 / 180.0 * M_PI);
        imu_raw.angular_velocity.set__y(package.imu.gyro_y / 32768.0 * 2000.0 / 180.0 * M_PI);
        imu_raw.angular_velocity.set__z(package.imu.gyro_z / 32768.0 * 2000.0 / 180.0 * M_PI);
        imu_raw.linear_acceleration.set__x(package.imu.accl_x / 32768.0 * 3.0 * 9.7944);
        imu_raw.linear_acceleration.set__y(package.imu.accl_y / 32768.0 * 3.0 * 9.7944);
        imu_raw.linear_acceleration.set__z(package.imu.accl_z / 32768.0 * 3.0 * 9.7944);

        imu_raw_publisher_->publish(imu_raw);

        skider_interface::msg::Sbus sbus;
        sbus.header.set__frame_id("joy_sbus_frame");
        sbus.header.set__stamp(gimbal_hw_node_->get_clock()->now());
        for (uint i = 0; i < 18; i++)
        {
            sbus.ch.push_back(package.sbus[i]);
        }

        sbus_publisher_->publish(sbus);
    }

    //---can0---

    // uint id = 0;
    // u_char buf[8] = {0}, buf_temp[2] = {0};
    // u_char dlc = 0;

    // while (1)
    // {

    //     if (this->can0_.receive(id, buf, dlc) == transporter_sdk::Can::READ_ERROR)
    //     {

    //         break;
    //     }

    //     switch (id)
    //     {

    //     case YAW:
    //     {

    //         buf_temp[0] = buf[1];
    //         buf_temp[1] = buf[0];
    //         memcpy(&(gimbal_state_msg_.yaw_angle), buf_temp, 2);
    //         device_online_msg_.yaw_motor = true;
    //         std::cout << "1" << std::endl;

    //         break;
    //     }
    //     case PITCH:
    //     {

    //         buf_temp[0] = buf[1];
    //         buf_temp[1] = buf[0];
    //         memcpy(&(gimbal_state_msg_.pitch_angle), buf_temp, 2);
    //         device_online_msg_.pitch_motor = true;
    //         std::cout << "2" << std::endl;

    //         break;
    //     }
    //     case AMMOR:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&(gimbal_state_msg_.ammor_speed), buf_temp, 2);
    //         // device_online_msg_.ammor_motor = true;
    //         std::cout << "3" << std::endl;

    //         break;
    //     }
    //     case AMMOL:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&(gimbal_state_msg_.ammol_speed), buf_temp, 2);
    //         // device_online_msg_.ammol_motor = true;
    //         std::cout << "4" << std::endl;

    //         break;
    //     }
    //     case ROTOR:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&(gimbal_state_msg_.rotor_speed), buf_temp, 2);
    //         // device_online_msg_.rotor_motor = true;
    //         std::cout << "5" << std::endl;

    //         break;
    //     }

    //     default:
    //         break;
    //     }
    // }
    // gimbal_state_msg_.header.frame_id = "gimbal_state_";
    // gimbal_state_msg_.header.stamp = gimbal_hw_node_->get_clock()->now();
    // gimbal_state_publisher_->publish(gimbal_state_msg_);

    //---can1---

    // int16_t speed;
    // for (int i = 0; i < 4; i++)
    // {

    //     this->can1_.receive(id, buf, dlc);

    //     switch (id)
    //     {
    //     case WHEEL1:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&speed, buf_temp, 2);
    //         chassis_state_msg_.speed[0] = speed;
    //         device_online_msg_.wheel1 = true;

    //         break;
    //     }

    //     case WHEEL2:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&speed, buf_temp, 2);
    //         chassis_state_msg_.speed[1] = speed;
    //         device_online_msg_.wheel2 = true;

    //         break;
    //     }

    //     case WHEEL3:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&speed, buf_temp, 2);
    //         chassis_state_msg_.speed[2] = speed;
    //         device_online_msg_.wheel3 = true;

    //         break;
    //     }

    //     case WHEEL4:
    //     {

    //         buf_temp[0] = buf[3];
    //         buf_temp[1] = buf[2];
    //         memcpy(&speed, buf_temp, 2);
    //         chassis_state_msg_.speed[3] = speed;
    //         device_online_msg_.wheel4 = true;

    //         break;
    //     }

    //     default:
    //         break;
    //     }
    // }

    // chassis_state_msg_.header.frame_id = "chassis_state";
    // chassis_state_msg_.header.stamp = gimbal_hw_node_->get_clock()->now();
    // chassis_state_publisher_->publish(chassis_state_msg_);

    // std::cout << "end:   " << gimbal_hw_node_->get_clock()->now().nanoseconds() << std::endl;
}

void GimbalHWNode::loop_send()
{

    // if(device_online_msg_.whole_robot){

    // std::cout<<"start:   "<<gimbal_hw_node_->get_clock()->now().nanoseconds()<<std::endl;

    // std::cout<<"send GIMBAL_COMMAND: "<<(int)buf_gimbal_[0]<<(int)buf_gimbal_[1]<<(int)buf_gimbal_[2]<<(int)buf_gimbal_[3]<<std::endl;
    // std::cout<<"send CHASSIS_COMMAND: "<<(int)buf_chassis_[0]<<(int)buf_chassis_[1]<<(int)buf_chassis_[2]<<(int)buf_chassis_[3]<<std::endl;

    this->can0_.send(GIMBAL_COMMAND, buf_gimbal_, sizeof(buf_gimbal_));
    this->can0_.send(SHOOT_COMMAND, buf_shooter_, sizeof(buf_shooter_));
    this->can1_.send(CHASSIS_COMMAND, buf_chassis_, sizeof(buf_chassis_));
    buf_gimbal_[8] = {0};
    buf_shooter_[8] = {0};
    buf_chassis_[8] = {0};
    // std::cout<<"end:     "<<gimbal_hw_node_->get_clock()->now().nanoseconds()<<std::endl;

    // }
    // if(device_online_msg_.whole_robot){

    //     this->can0_.send(SHOOT_COMMAND, buf_shooter_, sizeof(buf_shooter_));
    //     buf_shooter_[8] = {0};
    //     std::cout<<"send SHOOT_COMMAND"<<std::endl;

    // }
    // if(device_online_msg_.chassis){

    //     this->can1_.send(CHASSIS_COMMAND, buf_chassis_, sizeof(buf_chassis_));
    //     buf_chassis_[8] = {0};
    //     std::cout<<"send CHASSIS_COMMAND"<<std::endl;

    // }
    // if(device_online_msg_.gimbal){

    //     this->can0_.send(GIMBAL_COMMAND, buf_gimbal_, sizeof(buf_gimbal_));
    //     std::cout<<"send GIMBAL_COMMAND: "<<std::hex<<buf_gimbal_[0]<<buf_gimbal_[1]<<std::endl;
    //     buf_gimbal_[8] = {0};

    // }
    // std::cout<<gimbal_hw_node_->get_clock()->now().nanoseconds()<<std::endl;
    // std::cout<<stamp_.stamp.sec<<stamp_.stamp.nanosec<<"-"<<gimbal_hw_node_->get_clock()->now().nanoseconds()<<std::endl;
}

void GimbalHWNode::gimbal_command_msg_callback(const skider_interface::msg::GimbalCommand &msg)
{

    buf_gimbal_[8] = {0};
    buf_gimbal_[0] = (u_char)(msg.yaw_current >> 8);
    buf_gimbal_[1] = (u_char)(msg.yaw_current);
    buf_gimbal_[2] = (u_char)(msg.pitch_current >> 8);
    buf_gimbal_[3] = (u_char)(msg.pitch_current);
    // std::cout<<"yaw_current: "<<msg.yaw_current<<std::endl;

    buf_shooter_[8] = {0};
    buf_shooter_[0] = (u_char)(msg.ammor_current >> 8);
    buf_shooter_[1] = (u_char)(msg.ammor_current);
    buf_shooter_[2] = (u_char)(msg.ammol_current >> 8);
    buf_shooter_[3] = (u_char)(msg.ammol_current);
    buf_shooter_[4] = (u_char)(msg.rotor_current >> 8);
    buf_shooter_[5] = (u_char)(msg.rotor_current);
    // memcpy(buf_gimbal_, &msg.yaw_current, sizeof(msg.yaw_current));
}

void GimbalHWNode::chassis_command_msg_callback(const skider_interface::msg::ChassisCommand &msg)
{

    buf_chassis_[8] = {0};
    for (int i = 0; i < 4; i++)
    {

        buf_chassis_[2 * i] = (u_char)(msg.current[i] >> 8);
        // std::cout<<"msg.current[i]: "<<msg.current[i]<<std::endl;

        // std::cout<<"buf_chassis_[2*i]: "<<buf_chassis_[2*i]<<std::endl;

        buf_chassis_[2 * i + 1] = (u_char)(msg.current[i]);
    }
    // stamp_.stamp=msg.header.stamp;
}

void GimbalHWNode::loop_device_online()
{

    device_online_msg_.header.set__stamp(gimbal_hw_node_->get_clock()->now());
    // device_online_msg_.gimbal = device_online_msg_.yaw_motor&device_online_msg_.pitch_motor
    //                             &device_online_msg_.ammol_motor&device_online_msg_.ammor_motor
    //                             &device_online_msg_.rotor_motor;
    device_online_msg_.gimbal = device_online_msg_.yaw_motor & device_online_msg_.pitch_motor;
    device_online_msg_.chassis = device_online_msg_.wheel1 & device_online_msg_.wheel2 & device_online_msg_.wheel3 & device_online_msg_.wheel4;
    device_online_msg_.whole_robot = device_online_msg_.gimbal & device_online_msg_.chassis & device_online_msg_.sensor;
    device_online_publisher_->publish(device_online_msg_);

    device_online_msg_.yaw_motor = false;
    device_online_msg_.pitch_motor = false;
    // device_online_msg_.rotor_motor = false;
    // device_online_msg_.ammol_motor = false;
    // device_online_msg_.ammor_motor = false;

    device_online_msg_.wheel1 = false;
    device_online_msg_.wheel2 = false;
    device_online_msg_.wheel3 = false;
    device_online_msg_.wheel4 = false;

    device_online_msg_.sensor = false;
}

void GimbalHWNode::recevieCallBack()
{
    //---can0---
    while (rclcpp::ok())
    {
        auto start_time = gimbal_hw_node_->get_clock()->now().nanoseconds();
        std::cout << "start: " << start_time << std::endl;

        uint id = 0;
        // u_char buf[8] = {0};
        u_char buf[8] = {0}, buf_temp[2] = {0};
        u_char dlc = 0;
        // if (this->can0_.receive(id, buf, dlc) == transporter_sdk::Can::READ_ERROR)
        // {
        //     break;
        // }
        this->can0_.receive(id, buf, dlc);
        switch (id)
        {

        case YAW:
        {

            buf_temp[0] = buf[1];
            buf_temp[1] = buf[0];
            memcpy(&(gimbal_state_msg_.yaw_angle), buf_temp, 2);
            // device_online_msg_.yaw_motor = true;
            std::cout << "1" << std::endl;

            break;
        }
        case PITCH:
        {

            buf_temp[0] = buf[1];
            buf_temp[1] = buf[0];
            memcpy(&(gimbal_state_msg_.pitch_angle), buf_temp, 2);
            // device_online_msg_.pitch_motor = true;
            std::cout << "2" << std::endl;

            break;
        }
        case AMMOR:
        {

            buf_temp[0] = buf[3];
            buf_temp[1] = buf[2];
            memcpy(&(gimbal_state_msg_.ammor_speed), buf_temp, 2);
            // device_online_msg_.ammor_motor = true;
            std::cout << "3" << std::endl;

            break;
        }
        case AMMOL:
        {

            buf_temp[0] = buf[3];
            buf_temp[1] = buf[2];
            memcpy(&(gimbal_state_msg_.ammol_speed), buf_temp, 2);
            // device_online_msg_.ammol_motor = true;
            std::cout << "4" << std::endl;

            break;
        }
        case ROTOR:
        {

            buf_temp[0] = buf[3];
            buf_temp[1] = buf[2];
            memcpy(&(gimbal_state_msg_.rotor_speed), buf_temp, 2);
            // device_online_msg_.rotor_motor = true;
            std::cout << "5" << std::endl;

            break;
        }

        default:
            break;
        }

        gimbal_state_msg_.header.frame_id = "gimbal_state_";
        gimbal_state_msg_.header.stamp = gimbal_hw_node_->get_clock()->now();
        gimbal_state_publisher_->publish(gimbal_state_msg_);

        auto end_time = gimbal_hw_node_->get_clock()->now().nanoseconds();
        std::cout << "end:   " << end_time << std::endl;


        std::this_thread::sleep_for(std::chrono::microseconds((10*1000 - static_cast<int>(end_time - start_time))/1000));


    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto imu_sensor_node = std::make_shared<GimbalHWNode>();
    rclcpp::spin(imu_sensor_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
