#include"arm_motor_control.hpp"


ArmMotorControl::ArmMotorControl(rclcpp::Node::SharedPtr node):
m_ros_node(node)
{

    inIt();
    
}

int ArmMotorControl::inIt()
{

    m_can_bus1 = std::make_shared<CanBus>();
    m_can_bus2 = std::make_shared<CanBus>();

    if (!m_can_bus1->open()) {return -1;}
    
    if (!m_can_bus2->open(1)) {return -1;}

    m_motor_list.resize(6);
    
    m_motor_state.resize(m_motor_list.size());
    m_motor_pub_data.resiz(m_motor_pub_data.motor_id.size());
    
    //m_motor_list[0] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DMH6215,0x01,0x11);

    m_motor_list[0] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4340_48V,0x01,0x11);
    m_motor_list[1] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4340_48V,0x02,0x12);
    m_motor_list[2] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4340_48V,0x03,0x13);

    m_motor_list[3] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4310_48V,0x04,0x14);
    m_motor_list[4] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4310_48V,0x05,0x15);
    m_motor_list[5] = std::make_shared<DamiaoMotor> (m_can_bus1,DM_Motor_Type::DM4310_48V,0x06,0x16);

    motorDisable();

    setMotorMode(DamiaoCtrlMode::MIT_MODE);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    m_motor_cmd_service = m_ros_node->create_service<std_srvs::srv::SetBool>(
    "/robot/set_motor_zero_pos",
    std::bind(&ArmMotorControl::handleService, this, std::placeholders::_1, std::placeholders::_2));

    m_motor_state_pub =m_ros_node->create_publisher<sensor_msgs::msg::JointState>(
    "/robot/arm_joint_state",rclcpp::QoS(1).best_effort());

    m_motor_cmd_sub =m_ros_node ->create_subscription<robot_msg::msg::MotorCmd>("/robot/arm_joint_cmd",
    rclcpp::QoS(1).best_effort(),std::bind(&ArmMotorControl::motorCmdCallback, this, std::placeholders::_1));

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    m_motor_state_push_thread = std::thread(&ArmMotorControl::pubMotorState ,this);

    m_motor_cmd_send_thread =std::thread(&ArmMotorControl::pubMotorCmd ,this);

    
    return 0;
}


int ArmMotorControl::motorEnable(uint32_t motor_id)
{
    if(motor_id ==ALL_MOTORS)
    {

        for(auto &motor:m_motor_list)
        {
            motor->getControlMode();
            motor->enable();
        }
    }else
    {

        for(auto &motor:m_motor_list)
        {
            if(motor->getSlaveId() == motor_id)
            {
                motor->enable();
            }

        }  
    }
    

    std::this_thread::sleep_for(std::chrono::milliseconds(300));
        
    return 1 ;
}

int ArmMotorControl::motorDisable(uint32_t motor_id)
{

    if(motor_id == ALL_MOTORS)
    {
        for(auto &motor:m_motor_list)
        {
            motor->disable();
        }  

    }else
    {
        for(auto &motor:m_motor_list)
        {
            if(motor->getSlaveId() == motor_id)
            {
                motor->disable();
            }

        }  
    }
    return 1;
  
}


int ArmMotorControl::setMotorMode(DamiaoCtrlMode control_mode ,uint32_t motor_id)
{

    if(motor_id == ALL_MOTORS)
    {
        for(auto &motor : m_motor_list)
        {
            motor->setControlMode(control_mode);
        }      
    }else
    {
        for(size_t i = 0 ;i < m_motor_list.size() ; i++ )
        {
            if(m_motor_list[i]->getSlaveId() ==motor_id)
            {
                m_motor_list[i]->setControlMode(control_mode);
            }            

        }
    }

    return 1;
}



int ArmMotorControl::setMotorZero(uint32_t motor_id)
{
    if(motor_id == ALL_MOTORS)
    {
        for(auto &motor:m_motor_list)
        {
            motor->setZeroPosition();
        }
        return 1;  
    }
    else
    {
        for(auto &motor:m_motor_list)
        {
            if(motor->getSlaveId() == motor_id)
            {
            motor->setZeroPosition();
            return 1;
            }
        }

    }

    return -1;

}    

void ArmMotorControl::getMotorState()
{
 
    for(auto &motor:m_motor_list)
    {  
        motor->updateState();
        // std::cout<<"CAN ID电机错误码:"<< motor->getMotorState().state<<std::endl;   
    }
    
    std::lock_guard<std::mutex> lock(m_motor_state_mutx);
    for(size_t i =0 ;i < m_motor_list.size();i++)
    {
        m_motor_state.position[i] = m_motor_list[i]->getMotorState().pos;
        m_motor_state.velocity[i] = m_motor_list[i]->getMotorState().vel;
        m_motor_state.effort[i] = m_motor_list[i]->getMotorState().tau;
    }
            
}



void ArmMotorControl::pubMotorState()
{   rclcpp::Rate rate(100);
    
    while(rclcpp::ok())
    {
        getMotorState();//从缓存获取电机最新状态

        std::lock_guard<std::mutex> lock(m_motor_state_mutx);

        sensor_msgs::msg::JointState msg;

        for(size_t  i =0 ; i < m_motor_state.motor_num ; i++)
        {

            msg.name.push_back("joint" + std::to_string(i+1));

        }

        msg.header.stamp = m_ros_node->now();

        msg.position = m_motor_state.position;
        msg.velocity = m_motor_state.velocity;
        msg.effort = m_motor_state.effort;
    
        m_motor_state_pub->publish(msg);

        rate.sleep();
    }


}

void ArmMotorControl::pubMotorCmd()
{
    rclcpp::Rate rate(100);

    while (rclcpp::ok())
    {
        rate.sleep();

        std::lock_guard<std::mutex> lock(m_motor_pub_data_mutx);

        for (size_t i = 0; i < m_motor_pub_data.motor_id.size(); ++i)
        {
        
            uint32_t    motor_id = m_motor_pub_data.motor_id[i];
            int    mode     = m_motor_pub_data.control_mode[i];
            double pos      = m_motor_pub_data.pos[i];
            double vel      = m_motor_pub_data.vel[i];
            double eff      = m_motor_pub_data.eff[i];
            double kp       = m_motor_pub_data.kp[i];
            double kd       = m_motor_pub_data.kd[i];

            if(pos > m_motor_pub_data.pos_limit_max[i]){pos = m_motor_pub_data.pos_limit_max[i];}
            else if(pos < m_motor_pub_data.pos_limit_min[i]){pos = m_motor_pub_data.pos_limit_min[i];}

            if(eff > m_motor_pub_data.eff_limit_max[i]){eff = m_motor_pub_data.eff_limit_max[i];}
            else if(eff < m_motor_pub_data.eff_limit_min[i]){eff = m_motor_pub_data.eff_limit_min[i];}

            for (auto& motor : m_motor_list)
            {
                if (motor->getSlaveId() == motor_id)
                {
                    switch (mode)
                    {
                        case 0:
                            motor->disable();

                            break;

                        case 1:
                            if(motor->getMotorState().control_mode !=1)
                            {
                                motor->setControlMode(DamiaoCtrlMode::MIT_MODE);
                            }

                            if(motor->getMotorState().enabled ==false)
                            {
                                motor->enable();
                                
                            }
                            
                            motor->controlMIT(pos, vel, eff, kp, kd);
    
                            break;

                        case 2:
                            if(motor->getMotorState().control_mode !=2)
                            {
                                motor->setControlMode(DamiaoCtrlMode::POS_VEL_MODE);

                            }
                            
                            if(motor->getMotorState().enabled ==false)
                            {
                                motor->enable();
                                motor->setControlMode(DamiaoCtrlMode::POS_VEL_MODE);
                            }
    
                            motor->controlPosVel(pos, vel);
                            break;

                        case 3:
                            if(motor->getMotorState().control_mode !=3)
                            {
                                motor->setControlMode(DamiaoCtrlMode::VEL_MODE);
                            }
                            
                            if(motor->getMotorState().enabled ==false)
                            {
                                motor->enable();
                                motor->setControlMode(DamiaoCtrlMode::VEL_MODE);
                            }
                            motor->controlVel(vel);
                            break;

                        case 4:
                            {
                            if(motor->getMotorState().control_mode !=4)
                            {
                                motor->setControlMode(DamiaoCtrlMode::POS_FORCE_MODE);
                            }
                            
                            if(motor->getMotorState().enabled ==false)
                            {
                                motor->enable();
                                motor->setControlMode(DamiaoCtrlMode::POS_FORCE_MODE);
                            }

                            // 力位混控（位置 + 最大速度 + 范围0-1）
                            // 目标位置、限速、限流（0~1）
                            float vel_limit = std::abs(vel);    // 速度必须为正
                            float current_limit = std::abs(eff); // eff范围 0~1
                            current_limit = std::clamp(current_limit, 0.0f, 1.0f);

                            motor->controlPositionTorque(pos, vel_limit, current_limit);
                            }
                            break;

                        default:
                            RCLCPP_WARN(m_ros_node->get_logger(), "无效模式: %d", mode);
                    }

                    break;
                }
            }
        }

        m_motor_pub_data.pub_times++;
    }
}

void ArmMotorControl::motorCmdCallback(const robot_msg::msg::MotorCmd::SharedPtr msg)
{
    for (const auto& cmd : msg->commands)
    {
        setDataById(cmd.id, cmd.mode, cmd.position, cmd.velocity, cmd.torque ,cmd.kp ,cmd.kd);
    }
}


 void ArmMotorControl::handleService(
        const std_srvs::srv::SetBool::Request::SharedPtr request,
        std_srvs::srv::SetBool::Response::SharedPtr response)
{

    if(request->data ==true)
    {
        setMotorZero();
        response->success =true;
        response->message = "全部电机零点设置成功";

    }else
    {
        RCLCPP_INFO(m_ros_node-> get_logger(),"请求数据无效");
    }

}


 void ArmMotorControl::setDataById(uint32_t target_id, int mode, double position, double velocity, double effort, double kp_val, double kd_val)
{
    std::lock_guard<std::mutex> lock(m_motor_pub_data_mutx);
    
    for (size_t i = 0; i < m_motor_pub_data.motor_id.size(); ++i)
    {
        if (m_motor_pub_data.motor_id[i] == target_id)
        {
            m_motor_pub_data.control_mode[i] = mode;
            m_motor_pub_data.pos[i] = position;
            m_motor_pub_data.vel[i] = velocity;
            m_motor_pub_data.eff[i] = effort;
            m_motor_pub_data.kp[i] = kp_val;
            m_motor_pub_data.kd[i] = kd_val;
            
            return;
        }
    }
    m_motor_pub_data.pub_times =0;

    printf("⚠️  未找到电机ID: %d\n", (int)target_id);
}

ArmMotorControl::~ArmMotorControl()
{

    motorDisable();
    m_can_bus1->close();
    RCLCPP_INFO(m_ros_node->get_logger(), "✅ 程序关闭成功");

}