#pragma once
#include<mutex>
#include <chrono>
#include"rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "can_bus.hpp"
#include "damiao_motor.hpp"
#include "robot_msg/msg/motor_cmd.hpp"
#include "std_srvs/srv/set_bool.hpp"





class ArmMotorControl
{
public:

ArmMotorControl(rclcpp::Node::SharedPtr node);
~ArmMotorControl();
private:

int inIt();

int motorEnable(uint32_t motor_id = ALL_MOTORS);

int motorDisable(uint32_t motor_id = ALL_MOTORS);

int setMotorMode(DamiaoCtrlMode control_mode ,uint32_t motor_id = ALL_MOTORS);

// 无参全部设0点， 有参数对应ID设0点
int setMotorZero(uint32_t motor_id = ALL_MOTORS);

void getMotorState();

void pubMotorState();

void pubMotorCmd();

void handleService(const std_srvs::srv::SetBool::Request::SharedPtr request,
                    std_srvs::srv::SetBool::Response::SharedPtr response);

void motorCmdCallback(const robot_msg::msg::MotorCmd::SharedPtr msg);

void setDataById(uint32_t target_id, int mode = 0, double position = 0, double velocity = 0, double effort = 0, double kp_val =0, double kd_val =0);

private:

static constexpr int ALL_MOTORS = 999;


struct MotorPub
{
int pub_times =0 ;    
std::vector<uint32_t> motor_id = {0x01,0x02,0x03,0x04,0x05,0x06};
std::vector<int> control_mode;
std::vector<double> pos;
std::vector<double> vel;
std::vector<double> eff;
std::vector<double> kp;
std::vector<double> kd;

std::vector<double> pos_limit_max ={3.14, 3.14, 3.14, 3.14, 3.14, 3.14};
std::vector<double> pos_limit_min ={-3.14, -3.14, -3.14, -3.14, -3.14, -3.14};
std::vector<double> eff_limit_max ={ 2,  5,  3,  1,  1,  1};
std::vector<double> eff_limit_min ={-2, -5, -3, -1, -1, -1};

void resiz(size_t size)
{
    control_mode.resize(size);
    pos.resize(size);
    vel.resize(size);
    eff.resize(size);
    kp.resize(size);
    kd.resize(size);
}

};

struct MotorState
{

    std::vector<double> position;
    std::vector<double> velocity;
    std::vector<double> effort;   
    std::vector<float> temperature;
    std::vector<float> voltage;
    size_t motor_num =0;
    void resize(size_t size)
    {
        position.resize(size);
        velocity.resize(size);
        effort.resize(size);
        temperature.resize(size);
        voltage.resize(size);  
        motor_num = size;      
    }
};

rclcpp::Node::SharedPtr m_ros_node;

rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr m_motor_state_pub;

rclcpp::Subscription<robot_msg::msg::MotorCmd>::SharedPtr m_motor_cmd_sub;

rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr m_motor_cmd_service;

std::shared_ptr<CanBus> m_can_bus1;
std::shared_ptr<CanBus> m_can_bus2;

std::mutex m_motor_state_mutx;

std::mutex m_motor_pub_data_mutx;

std::thread m_motor_state_push_thread;

std::thread m_motor_cmd_send_thread;

std::vector<std::shared_ptr<DamiaoMotor>> m_motor_list;

MotorState m_motor_state;
MotorPub m_motor_pub_data;

};