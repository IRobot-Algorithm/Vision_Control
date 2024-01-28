#include "gimbal_demo_node.hpp"
#include "../../skider_utils/include/utils.hpp"

GimbalControlerDemoNode::GimbalControlerDemoNode(const rclcpp::NodeOptions &options)
{
    gimbal_controler_demo_node_ = std::make_shared<rclcpp::Node>("gimbal_controler_node", options);
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Node Begin");

    // kp, ki, kd
    std::map<std::string, std::vector<double>> pid_params{
        {"pid_yaw_remote_in", {0.0, 0.0, 0, 0}},
        {"pid_yaw_remote_out", {0.0, 0.0, 0.0}},
        {"pid_yaw_vision_in", {0.0, 0.0, 0, 0}},
        {"pid_yaw_vision_out", {0.0, 0.0, 0.0}},
        {"pid_yaw_init_in", {0.0, 0.0, 0, 0}},
        {"pid_yaw_init_out", {0.0, 0.0, 0.0}},
        {"pid_pitch_remote_in", {0.0, 0.0, 0.0}},
        {"pid_pitch_remote_out", {0.0, 0.0, 0.0}},
        {"pid_pitch_vision_in", {0.0, 0.0, 0.0}},
        {"pid_pitch_vision_out", {0.0, 0.0, 0.0}},
        {"pid_ammor", {0.0, 0.0, 0.0}},
        {"pid_ammol", {0.0, 0.0, 0.0}},
        {"pid_rotor", {0.0, 0.0, 0.0}},

    };

    std::map<std::string, double> double_params{

        {"ammo_goal_speed", 0.0},
        {"rotor_goal_speed", 0.0},

    };

    std::map<std::string, bool> bool_params{

        {"ammo_enable", false},
    };

    // pid parameter
    gimbal_controler_demo_node_->declare_parameters("", pid_params);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_remote_in", pid_yaw_remote_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_remote_out", pid_yaw_remote_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_init_in", pid_yaw_init_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_init_out", pid_yaw_init_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_vision_in", pid_yaw_vision_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_yaw_vision_out", pid_yaw_vision_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_remote_in", pid_pitch_remote_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_remote_out", pid_pitch_remote_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_vision_in", pid_pitch_vision_in_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_pitch_vision_out", pid_pitch_vision_out_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_ammor", pid_ammor_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_ammol", pid_ammol_params_);
    gimbal_controler_demo_node_->get_parameter<std::vector<double>>("pid_rotor", pid_rotor_params_);
    std::cout << " pid_yaw_remote_in_params_[0]: " << pid_yaw_remote_in_params_[0] << " pid_yaw_remote_in_params_[1]: " << pid_yaw_remote_in_params_[1] << " pid_yaw_remote_in_params_[2]: " << pid_yaw_remote_in_params_[2] << std::endl;

    // double parameter
    gimbal_controler_demo_node_->declare_parameters("", double_params);
    gimbal_controler_demo_node_->get_parameter<double>("ammo_goal_speed", ammo_goal_speed_);
    gimbal_controler_demo_node_->get_parameter<double>("rotor_goal_speed", rotor_goal_speed_);
    std::cout << " ammo_goal_speed_: " << ammo_goal_speed_ << " rotor_goal_speed_: " << rotor_goal_speed_ << std::endl;

    // bool parameter
    gimbal_controler_demo_node_->declare_parameters("", bool_params);
    gimbal_controler_demo_node_->get_parameter<bool>("ammo_enable", ammo_enable_);
    std::cout << " ammo_enable_: " << ammo_enable_ << std::endl;

    std::string imu_subscribe_topic_name_("/skider/imu/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe IMU data : \"%s\"", imu_subscribe_topic_name_.c_str());
    imu_subscription_ = gimbal_controler_demo_node_->create_subscription<skider_interface::msg::Imu>(
        imu_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::imu_msg_callback, this, std::placeholders::_1));

    std::string joy_subscribe_topic_name_("/skider/joy/data");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Subscribe JOY data : \"%s\"", joy_subscribe_topic_name_.c_str());
    joy_subscription_ = gimbal_controler_demo_node_->create_subscription<sensor_msgs::msg::Joy>(
        joy_subscribe_topic_name_, 10, std::bind(&GimbalControlerDemoNode::joy_msg_callback, this, std::placeholders::_1));

    gimbal_state_subscription_ = gimbal_controler_demo_node_->create_subscription<skider_interface::msg::GimbalState>(
        "/skider/state/gimbal", 10, std::bind(&GimbalControlerDemoNode::gimbal_msg_callback, this, std::placeholders::_1));

    autoaim_target_subscription_ = gimbal_controler_demo_node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/rmos/autoaim/target", 10, std::bind(&GimbalControlerDemoNode::autoaim_msg_callback, this, std::placeholders::_1));

    std::string gimbal_command_publish_topic_name_("/skider/command/gimbal");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Gimbal Command Publisher : ");
    gimbal_command_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_interface::msg::GimbalCommand>(
        gimbal_command_publish_topic_name_, 10);

    std::string gimbal_debug_publisg_topic_name_("/skider/debug/gimbal");
    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Init Debug Publisher : ");
    gimbal_debug_publisher_ = gimbal_controler_demo_node_->create_publisher<skider_interface::msg::GimbalDebug>(
        gimbal_debug_publisg_topic_name_, 10);

    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Follow Init Timer : ");
    follow_init_timer_ = gimbal_controler_demo_node_->create_wall_timer(1000ms, [this]()
                                                                        { follow_init_ = true; });
    calculate_call_backgroup_ = gimbal_controler_demo_node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    calculate_pid_timer_ = gimbal_controler_demo_node_->create_wall_timer(1ms, std::bind(&GimbalControlerDemoNode::loop_calculate, this), calculate_call_backgroup_);

    RCLCPP_INFO(gimbal_controler_demo_node_->get_logger(), "Finish Init");

    this->pid_yaw_remote_in_ = PID(pid_yaw_remote_in_params_[0], pid_yaw_remote_in_params_[1], pid_yaw_remote_in_params_[2]);
    this->pid_yaw_remote_in_.i_sum_limit_ = Limit(-5000, 5000);
    this->pid_yaw_remote_out_ = PID(pid_yaw_remote_out_params_[0], pid_yaw_remote_out_params_[1], pid_yaw_remote_out_params_[2]);
    this->pid_yaw_init_in_ = PID(pid_yaw_init_in_params_[0], pid_yaw_init_in_params_[1], pid_yaw_init_in_params_[2]);
    this->pid_yaw_init_out_ = PID(pid_yaw_init_out_params_[0], pid_yaw_init_out_params_[1], pid_yaw_init_out_params_[2]);
    this->pid_yaw_vision_in_ = PID(pid_yaw_vision_in_params_[0], pid_yaw_vision_in_params_[1], pid_yaw_vision_in_params_[2]);
    this->pid_yaw_vision_out_ = PID(pid_yaw_vision_out_params_[0], pid_yaw_vision_out_params_[1], pid_yaw_vision_out_params_[2]);
    this->pid_pitch_remote_in_ = PID(pid_pitch_remote_in_params_[0], pid_pitch_remote_in_params_[1], pid_pitch_remote_in_params_[2]);
    this->pid_pitch_remote_out_ = PID(pid_pitch_remote_out_params_[0], pid_pitch_remote_out_params_[1], pid_pitch_remote_out_params_[2]);
    this->pid_pitch_vision_in_ = PID(pid_pitch_vision_in_params_[0], pid_pitch_vision_in_params_[1], pid_pitch_vision_in_params_[2]);
    this->pid_pitch_vision_out_ = PID(pid_pitch_vision_out_params_[0], pid_pitch_vision_out_params_[1], pid_pitch_vision_out_params_[2]);
    this->pid_ammor_ = PID(pid_ammor_params_[0], pid_ammor_params_[1], pid_ammor_params_[2]);
    this->pid_ammol_ = PID(pid_ammol_params_[0], pid_ammol_params_[1], pid_ammol_params_[2]);
    this->pid_rotor_ = PID(pid_rotor_params_[0], pid_rotor_params_[1], pid_rotor_params_[2]);
}

inline double speed_limit(double input, double max)
{
    if (input > max)
    {
        return max;
    }
    else if (input < -max)
    {
        return -max;
    }
    return input;
}

inline double get_relative_angle(double angle_aim, double angle_ref, int type)
{
    double reletive_angle = angle_aim - angle_ref;

    // 弧度
    if (type == 1)
    {
        while (reletive_angle > M_PI)
        {
            reletive_angle -= 2 * M_PI;
        }
        while (reletive_angle < -M_PI)
        {
            reletive_angle += 2 * M_PI;
        }
    }

    // 机械角度
    if (type == 2)
    {

        while (reletive_angle > 4096)
        {
            reletive_angle -= 2 * 4096;
        }
        while (reletive_angle < -4096)
        {
            reletive_angle += 2 * 4096;
        }
    }
    return reletive_angle;
}

inline double aim_loop(double angle_aim)
{
    while (angle_aim > M_PI)
    {
        angle_aim -= 2 * M_PI;
    }
    while (angle_aim < -M_PI)
    {
        angle_aim += 2 * M_PI;
    }

    return angle_aim;
}

inline double aim_limut(double angle_aim, double max, double min)
{
    while (angle_aim > max)
    {
        return max;
    }
    while (angle_aim < min)
    {
        return min;
    }

    return angle_aim;
}

void GimbalControlerDemoNode::joy_msg_callback(const sensor_msgs::msg::Joy &msg)
{

    // std::cout << "start" << gimbal_controler_demo_node_->get_clock()->now().nanoseconds() << std::endl;

    // gimbal_command_msg_.header.set__frame_id("Controler Gimbal Command");
    // gimbal_command_msg_.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());

    // 云台有力
    // if ((msg.buttons[1] == true) || (msg.buttons[2] == true) || (msg.buttons[4] != true))
    // {

    //     if (follow_init_ != true)
    //     {

    //         // 云台转到底盘处
    //         double yaw_relative = get_relative_angle(yaw_zero_angle_, yaw_angle_, 2);
    //         double yaw_init = yaw_angle_ + yaw_relative;

    //         double yaw_w_goal = this->pid_yaw_init_out_.calculate(yaw_init, yaw_angle_);
    //         double yaw_current = this->pid_yaw_init_in_.calculate(yaw_w_goal, w_yaw_);
    //         // gimbal_command_msg_.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));

    //         yaw_angle_set_ = imu_yaw_;
    //         // gimbal_command_msg_.follow_init = follow_init_;
    //     }
    //     else
    //     {

    //         yaw_angle_set_ = aim_loop(yaw_angle_set_ + (-msg.axes[2]) * 0.01);

    //         double yaw_relative = get_relative_angle(yaw_angle_set_, imu_yaw_, 1);
    //         yaw_angle_set_ = imu_yaw_ + yaw_relative;
    //         // std::cout<<"yaw_angle_set_: "<<yaw_angle_set_<<" imu_yaw_: "<<imu_yaw_<<" yaw_relative: "<<yaw_relative<<std::endl;

    //         double yaw_w_goal = this->pid_yaw_remote_out_.calculate(yaw_angle_set_, imu_yaw_);
    //         double yaw_current = this->pid_yaw_remote_in_.calculate(yaw_w_goal, w_yaw_);

    //         // gimbal_command_msg_.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));

    //         // gimbal_command_msg_.follow_init = true;
    //     }

    //     pitch_angle_set_ = aim_limut((pitch_angle_set_ + (msg.axes[3]) * 0.03), 0.43, -0.4);
    //     double pitch_w_goal = this->pid_pitch_remote_out_.calculate(pitch_angle_set_, imu_pitch_);
    //     double pitch_current = this->pid_pitch_remote_in_.calculate(pitch_w_goal, w_pitch_);
    //     // gimbal_command_msg_.pitch_current = (int16_t)(speed_limit(pitch_current, 30000));
    // }

    // 摩擦轮转动TOCHECK
    // if (msg.buttons[2] == true)
    // {

    //     if (ammo_enable_)
    //     {
    //         gimbal_command_msg_.ammor_current = this->pid_ammor_.calculate(ammo_goal_speed_, ammor_speed_);
    //         gimbal_command_msg_.ammol_current = this->pid_ammol_.calculate(-ammo_goal_speed_, ammol_speed_);
    //     }
    // }
    // rotor
    if (msg.axes[4] > 0.9f && msg.buttons[2] == true)
    {
        rotor_enable_ = true;
        // gimbal_command_msg_.rotor_current = this->pid_rotor_.calculate(rotor_goal_speed_, rotor_speed_);
        // // gimbal_command_msg_.rotor_current = this->pid_rotor_.calculate(rotor_goal_speed_, rotor_speed_) + 300 + rotor_goal_speed_ * 0.05;
        // gimbal_command_msg_.rotor_current = (int16_t)(speed_limit(gimbal_command_msg_.rotor_current, 30000));
    }
    // else if (msg.buttons[2] != true)
    // {

    //     gimbal_command_msg_.rotor_current = 0;
    // }

    // auto aim
    // if (msg.buttons[4] == true)
    // {

    //     yaw_angle_set_ = aim_loop(yaw_angle_set_ + autoaim_yaw_);

    //     double yaw_relative = get_relative_angle(yaw_angle_set_, imu_yaw_, 1);
    //     yaw_angle_set_ = imu_yaw_ + yaw_relative;

    //     double yaw_w_goal = this->pid_yaw_vision_out_.calculate(yaw_angle_set_, imu_yaw_);
    //     double yaw_current = this->pid_yaw_vision_in_.calculate(yaw_w_goal, w_yaw_);
    //     // std::cout<<yaw_w_goal<<"\t"<<w_yaw_<<"\t"<<yaw_current<<std::endl;
    //     gimbal_command_msg_.yaw_current = (int16_t)((int)(speed_limit(yaw_current, 30000)));
    //     gimbal_command_msg_.follow_init = true;

    //     pitch_angle_set_ = aim_limut(autoaim_pitch_, 0.25, -0.4);
    //     double pitch_w_goal = this->pid_pitch_vision_out_.calculate(pitch_angle_set_, imu_pitch_);
    //     double pitch_current = this->pid_pitch_vision_in_.calculate(pitch_w_goal, w_pitch_);
    //     gimbal_command_msg_.pitch_current = (int16_t)(speed_limit(pitch_current, 30000));

    // }

    if (msg.buttons[0] == true)
    {

        gimbal_command_msg_.yaw_current = 0;
        gimbal_command_msg_.pitch_current = 0;
        gimbal_command_msg_.ammor_current = 0;
        gimbal_command_msg_.ammol_current = 0;
        gimbal_command_msg_.rotor_current = 0;
        gimbal_command_msg_.follow_init = false;

        follow_init_timer_->reset();
        follow_init_ = false;

        rotor_enable_ = false;

    }
    // debug_publisher_->publish(debug_msg_);

    // debug
    // gimbal_debug_msg_.header.set__frame_id("debug_frame");
    // gimbal_debug_msg_.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    // gimbal_debug_msg_.yaw_angle_input = yaw_angle_set_;
    // gimbal_debug_msg_.yaw_angle_state = imu_yaw_;
    // // gimbal_debug_msg_.yaw_w_input = yaw_w_goal;
    // gimbal_debug_msg_.yaw_w_state = w_yaw_;
    // gimbal_debug_msg_.pitch_angle_input = pitch_angle_set_;
    // gimbal_debug_msg_.pitch_angle_state = imu_pitch_;
    // // gimbal_debug_msg_.pitch_w_input = pitch_w_goal;
    // gimbal_debug_msg_.pitch_w_state = w_pitch_;
    // gimbal_debug_msg_.rotor_speed_input = rotor_goal_speed_;
    // gimbal_debug_msg_.rotor_speed_state = rotor_speed_;
    // gimbal_debug_publisher_->publish(gimbal_debug_msg_);

    // gimbal_command_publisher_->publish(gimbal_command_msg_);
    // gimbal_command_msg_.yaw_current = 0;
    // gimbal_command_msg_.pitch_current = 0;
    // gimbal_command_msg_.ammor_current = 0;
    // gimbal_command_msg_.ammol_current = 0;
    // gimbal_command_msg_.rotor_current = 0;
    // gimbal_command_msg_.follow_init = false;

    // std::cout << "end  " << gimbal_controler_demo_node_->get_clock()->now().nanoseconds() << std::endl;
}

void GimbalControlerDemoNode::imu_msg_callback(const skider_interface::msg::Imu &msg)
{

    imu_yaw_ = msg.imu_yaw;
    imu_pitch_ = msg.imu_pitch;
    imu_roll_ = msg.imu_roll;

    w_pitch_ = msg.imu.angular_velocity.y;
    w_yaw_ = msg.imu.angular_velocity.z;
}

void GimbalControlerDemoNode::gimbal_msg_callback(const skider_interface::msg::GimbalState &msg)
{
    // 不使用
    yaw_angle_ = msg.yaw_angle;
    pitch_angle_ = msg.pitch_angle;

    // 使用
    ammor_speed_ = msg.ammor_speed;
    ammol_speed_ = msg.ammol_speed;
    rotor_speed_ = msg.rotor_speed;
}

void GimbalControlerDemoNode::autoaim_msg_callback(const geometry_msgs::msg::Vector3 &msg)
{

    autoaim_pitch_ = msg.x;
    autoaim_yaw_ = msg.y;
    autoaim_roll_ = msg.z;
    std::cout << " autoaim_pitch_: " << autoaim_pitch_ << " autoaim_yaw_: " << autoaim_yaw_ << std::endl;
}

void GimbalControlerDemoNode::loop_calculate()
{
    std::cout << "start" << gimbal_controler_demo_node_->get_clock()->now().nanoseconds() << std::endl;
    if(rotor_enable_){

        std::cout << "rotor_speed_" << rotor_speed_ << std::endl;

        gimbal_command_msg_.rotor_current = this->pid_rotor_.calculate(rotor_goal_speed_, rotor_speed_);
        std::cout << "rotor_speed_" << rotor_speed_ << std::endl;

        gimbal_command_msg_.rotor_current = (int16_t)(speed_limit(gimbal_command_msg_.rotor_current, 30000));
        // gimbal_command_publisher_->publish(gimbal_command_msg_);
        gimbal_command_msg_.yaw_current = 0;
        gimbal_command_msg_.pitch_current = 0;
        gimbal_command_msg_.ammor_current = 0;
        gimbal_command_msg_.ammol_current = 0;
        // gimbal_command_msg_.rotor_current = 0;
        gimbal_command_msg_.follow_init = false;

    }
    else{
        gimbal_command_msg_.rotor_current = 0;

    }
    gimbal_command_msg_.header.set__frame_id("Controler Gimbal Command");
    gimbal_command_msg_.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    gimbal_command_publisher_->publish(gimbal_command_msg_);

    gimbal_debug_msg_.header.set__frame_id("debug_frame");
    gimbal_debug_msg_.header.set__stamp(gimbal_controler_demo_node_->get_clock()->now());
    gimbal_debug_msg_.rotor_speed_input = rotor_goal_speed_;
    gimbal_debug_msg_.rotor_speed_state = rotor_speed_;
    gimbal_debug_publisher_->publish(gimbal_debug_msg_);
    std::cout << "end  " << gimbal_controler_demo_node_->get_clock()->now().nanoseconds() << std::endl;

}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto gimbal_controler_demo_node = std::make_shared<GimbalControlerDemoNode>();
    rclcpp::spin(gimbal_controler_demo_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
