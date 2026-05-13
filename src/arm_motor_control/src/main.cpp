#include<iostream>
#include"rclcpp/rclcpp.hpp"
#include"arm_motor_control.hpp"

int main(int argc, char *argv[])
{
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<rclcpp::Node> ("arm_motor_control"); 

    auto arm_motor_control = std::make_shared<ArmMotorControl>(node);

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}