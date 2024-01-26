#include "chassis_demo_node.hpp"

ChassisControlerDemoNode::ChassisControlerDemoNode(const rclcpp::NodeOptions & options)
{
    chassis_controler_demo_node_ = std::make_shared<rclcpp::Node>("chassis_controler_node", options);
    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Node Begin");


    //kp, ki, kd
    std::map<std::string, std::vector<double>> pid_params {
        {"pid1", {0.0, 0.0, 0,0}},
        {"pid2", {0.0, 0.0, 0.0}},
        {"pid3", {0.0, 0.0, 0.0}},
        {"pid4", {0.0, 0.0, 0.0}},
        {"pid_follow", {0.0, 0.0, 0.0}},

    };
    std::map<std::string, double> params{

        {"spin_w", 0.0},
        {"yaw_zero_angle", 0.0},

    };

    chassis_controler_demo_node_->declare_parameters("", pid_params);
    // // pid parameter
    chassis_controler_demo_node_->get_parameter<std::vector<double>>("pid1", pid1_params_);
    chassis_controler_demo_node_->get_parameter<std::vector<double>>("pid2", pid2_params_);
    chassis_controler_demo_node_->get_parameter<std::vector<double>>("pid3", pid3_params_);
    chassis_controler_demo_node_->get_parameter<std::vector<double>>("pid4", pid4_params_);
    chassis_controler_demo_node_->get_parameter<std::vector<double>>("pid_follow", pid_follow_params_);

    std::cout<<" pid1_params_[0]: "<<pid1_params_[0]<<" pid1_params_[1]: "<<pid1_params_[1]<<" pid1_params_[2]: "<<pid1_params_[2]<<std::endl;

    chassis_controler_demo_node_->declare_parameters("", params);
    chassis_controler_demo_node_->get_parameter<double>("spin_w", spin_w_);
    chassis_controler_demo_node_->get_parameter<double>("yaw_zero_angle", yaw_zero_angle_);
    
    std::cout<<" spin_w_: "<<spin_w_<<" yaw_zero_angle_: "<<yaw_zero_angle_<<std::endl;

    // ---subscription---
    std::string imu_subscribe_topic_name_("/skider/imu/data");
    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Subscribe IMU data : \"%s\"", imu_subscribe_topic_name_.c_str());
    imu_subscription_ = chassis_controler_demo_node_->create_subscription<skider_interface::msg::Imu>(
        imu_subscribe_topic_name_, 10, std::bind(&ChassisControlerDemoNode::imu_msg_callback, this, std::placeholders::_1));

    std::string joy_subscribe_topic_name_("/skider/joy/data");
    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Subscribe JOY data : \"%s\"", joy_subscribe_topic_name_.c_str());
    joy_subscription_ = chassis_controler_demo_node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_subscribe_topic_name_, 10, std::bind(&ChassisControlerDemoNode::joy_msg_callback, this, std::placeholders::_1));

    chassis_state_subscription_ = chassis_controler_demo_node_->create_subscription<skider_interface::msg::ChassisState>(
        "/skider/state/chassis", 10, std::bind(&ChassisControlerDemoNode::chassis_msg_callback, this, std::placeholders::_1));

    gimbal_state_subscription_ = chassis_controler_demo_node_->create_subscription<skider_interface::msg::GimbalState>(
        "/skider/state/gimbal", 10, std::bind(&ChassisControlerDemoNode::gimbal_msg_callback, this, std::placeholders::_1));
        
    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Subscribe Gimbal Command");
    gimbal_command_subscription_ = chassis_controler_demo_node_->create_subscription<skider_interface::msg::GimbalCommand>(
        "/skider/command/gimbal", 10, std::bind(&ChassisControlerDemoNode::gimbal_command_msg_callback, this, std::placeholders::_1));

    // ---publisher---
    std::string chassis_command_publish_topic_name_("/skider/command/chassis");
    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Init Chassis Command Publisher : ");
    chassis_command_publisher_ = chassis_controler_demo_node_->create_publisher<skider_interface::msg::ChassisCommand>(
        chassis_command_publish_topic_name_, 10);



    // ---debug---
    // std::string chassis_debug_publisg_topic_name_("/skider/debug");
    // RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Init Debug Publisher : ");
    // debug_publisher_ = chassis_controler_demo_node_->create_publisher<skider_interface::msg::Debug>(
    //     chassis_debug_publisg_topic_name_, 10);

    // ---timer---
    timer_1000Hz_ = chassis_controler_demo_node_->create_wall_timer(100us, std::bind(&ChassisControlerDemoNode::loop_10000Hz, this));

    RCLCPP_INFO(chassis_controler_demo_node_->get_logger(), "Finish Init");

    // ---PID---
    PID pid1(pid1_params_[0], pid1_params_[1], pid1_params_[2]);
    pid1.i_sum_limit_ = Limit(-1000, 1000);
    PID pid2(pid2_params_[0], pid2_params_[1], pid2_params_[2]);
    pid2.i_sum_limit_ = Limit(-1000, 1000);
    PID pid3(pid3_params_[0], pid3_params_[1], pid3_params_[2]);
    pid3.i_sum_limit_ = Limit(-1000, 1000);
    PID pid4(pid4_params_[0], pid4_params_[1], pid4_params_[2]);
    pid4.i_sum_limit_ = Limit(-1000, 1000);

    pid_follow_ = PID(pid_follow_params_[0], pid_follow_params_[1], pid_follow_params_[2]);
    pid_follow_.i_sum_limit_ = Limit(-1000, 1000);

    pid_vec_.push_back(pid1);
    pid_vec_.push_back(pid2);
    pid_vec_.push_back(pid3);
    pid_vec_.push_back(pid4);


}

inline double speed_limit(double input, double max)
{
    if (input > max) {
        return max;
    }
    else if (input < -max) {
        return -max;
    }
    return input;
}

inline double get_relative_angle(double angle_aim, double angle_ref)
{
    double reletive_angle = angle_aim - angle_ref;

    while (reletive_angle > 4096) {
        reletive_angle -= 2*4096;
    }
    while (reletive_angle < -4096) {
        reletive_angle += 2*4096;
    }

    return reletive_angle;
}

inline double aim_loop(double angle_aim)
{
    while (angle_aim > M_PI) {
        angle_aim -= 2*M_PI;
    }
    while (angle_aim < -M_PI) {
        angle_aim += 2*M_PI;
    }

    return angle_aim;
}

inline double aim_limut(double angle_aim, double max, double min)
{
    while (angle_aim > max) {
        return max;
    }
    while (angle_aim < min) {
        return min;
    }

    return angle_aim;
}


void ChassisControlerDemoNode::loop_10000Hz()
{

    // command send
    skider_interface::msg::ChassisCommand chassis_msg;
    chassis_msg.header.set__frame_id("Controler Chassis Command");
    //chassis_msg.header.set__stamp(chassis_controler_demo_node_->get_clock()->now());
    chassis_msg.header.stamp=stamp_.stamp;
    double chassis_current[4];

    // 底盘有力
    if ((button1_ == true)  ||  (button2_ == true)){       

        if(follow_init_ != true){

            for(int i=0; i<4; i++){
            
            chassis_msg.current.push_back(0);
            }

        }
        else{


            double yaw_relative = get_relative_angle(yaw_zero_angle_, yaw_angle_);
            double yaw_angle_set_ = yaw_angle_ + yaw_relative;

            if(axes4_ < -0.9){

                follow_w_ = spin_w_;
            }
            else{

                follow_w_ = pid_follow_.calculate(yaw_angle_set_, yaw_angle_);

            }
            //std::cout<<"follow_w_: "<<follow_w_<<std::endl;


            double delta_yaw = yaw_relative / 4096 * M_PI;
            vx_solve_ = vx_set_*cos(delta_yaw) - vy_set_*sin(delta_yaw);
            vy_solve_ = -vx_set_*sin(delta_yaw) + vy_set_*cos(delta_yaw);




            chassis_speed_[0] = (-vx_solve_ - vy_solve_ + follow_w_);
            chassis_speed_[1] = (vx_solve_ - vy_solve_ + follow_w_);
            chassis_speed_[2] = (vx_solve_ + vy_solve_ + follow_w_);
            chassis_speed_[3] = (-vx_solve_ + vy_solve_ + follow_w_);


            for(int i=0; i<4; i++){

                chassis_current[i] = pid_vec_[i].calculate_robust(chassis_speed_[i], chassis_state_[i]);
                chassis_current[i] = speed_limit(chassis_current[i], 16384);
                chassis_msg.current.push_back(chassis_current[i]);

            }

        }

    }
    // 底盘无力
    else{

        for(int i=0; i<4; i++){
            chassis_msg.current.push_back(0);

        }
    }
    chassis_command_publisher_->publish(chassis_msg);
    // debug_msg_.header.stamp = chassis_controler_demo_node_->get_clock()->now();
    // debug_msg_.header.frame_id = "debug";


}


void ChassisControlerDemoNode::joy_msg_callback(const sensor_msgs::msg::Joy & msg)
{
    button1_= msg.buttons[1];
    button2_= msg.buttons[2];
    if ((msg.buttons[1] == true)  ||  (msg.buttons[2] == true)){       

        vx_set_ = msg.axes[1]*6000;
        vy_set_ = -msg.axes[0]*6000;

    }
    else{
       

        vx_set_ =0;
        vy_set_ =0;
    }
    axes4_ = msg.axes[4];

    // int debug_index = 0;
    // debug_msg_.input1 = chassis_speed_[debug_index];
    // debug_msg_.state1 = chassis_state_[debug_index];
    // debug_publisher_->publish(debug_msg_);
}


void ChassisControlerDemoNode::imu_msg_callback(const skider_interface::msg::Imu & msg)
{
    imu_yaw_ = msg.imu_yaw;



}

void ChassisControlerDemoNode::chassis_msg_callback(const skider_interface::msg::ChassisState & msg)
{
    stamp_=msg.header;
    chassis_state_[0] = msg.speed[0];
    chassis_state_[1] = msg.speed[1];
    chassis_state_[2] = msg.speed[2];
    chassis_state_[3] = msg.speed[3];


}

void ChassisControlerDemoNode::gimbal_msg_callback(const skider_interface::msg::GimbalState & msg)
{

    //TOCHECK
    
    yaw_angle_ = msg.yaw_angle;

    //不用    
    pitch_angle_ = msg.pitch_angle;
    ammor_speed_ = msg.ammor_speed;
    ammol_speed_ = msg.ammol_speed;
    rotor_speed_ = msg.rotor_speed;


}

void ChassisControlerDemoNode::gimbal_command_msg_callback(const skider_interface::msg::GimbalCommand & msg){

    follow_init_ = msg.follow_init;


}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto chassis_controler_demo_node = std::make_shared<ChassisControlerDemoNode>();
    rclcpp::spin(chassis_controler_demo_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}


