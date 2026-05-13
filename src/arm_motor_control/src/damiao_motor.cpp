#include "damiao_motor.hpp"
#include <cmath>
#include <cstring>
#include <algorithm>



const MotorLimit motor_limit_table[Num_Of_Motor] = {
    {12.5f,  30.0f,  10.0f },    // DM4310
    {12.5f,  50.0f,  10.0f },    // DM4310_48V
    {12.5f,   8.0f,  28.0f },    // DM4340
    {12.5f,  20.0f,  28.0f },    // DM4340_48V
    {12.5f,  45.0f,  20.0f },    // DM6006
    {12.5f,  45.0f,  40.0f },    // DM8006
    {12.5f,  45.0f,  54.0f },    // DM8009
    {12.5f,  25.0f, 200.0f },    // DM10010L
    {12.5f,  20.0f, 200.0f },    // DM10010
    {12.5f, 280.0f,   1.0f },    // DMH3510
    {12.5f,  45.0f,  10.0f },    // DMH6215
    {12.5f,  45.0f,  10.0f },    // DMG6220
};


DamiaoMotor::DamiaoMotor(std::shared_ptr<CanBus> can_bus ,DM_Motor_Type type, uint8_t slave_id, uint8_t master_id)
    : slave_id_(slave_id),
        master_id_(master_id),
        m_can_bus(can_bus)
           
{

    limit_ = motor_limit_table[type];
}


uint16_t DamiaoMotor::floatToUint(float x, float min, float max, uint8_t bits)
{
    float span = max - min;
    float norm = (x - min) / span;

    if (norm < 0.0f) norm = 0.0f;
    if (norm > 1.0f) norm = 1.0f;

    uint16_t max_val = (1 << bits) - 1;
    return static_cast<uint16_t>(norm * max_val);
}


float DamiaoMotor::uintToFloat(uint16_t x, float min, float max, uint8_t bits)
{
    uint16_t max_val = (1 << bits) - 1;
    float norm = static_cast<float>(x) / max_val;
    return min + norm * (max - min);
}


void DamiaoMotor::enable()
{
    tx_id_ = slave_id_;
    memset(tx_data_, 0xFF, 8);
    tx_data_[7] = 0xFC;
    sendCAN(); 
}


void DamiaoMotor::disable()
{
    tx_id_ = slave_id_;
    memset(tx_data_, 0xFF, 8);
    tx_data_[7] = 0xFD;
    sendCAN(); 
}


void DamiaoMotor::setZeroPosition()
{
    tx_id_ = slave_id_;
    memset(tx_data_, 0xFF, 8);
    tx_data_[7] = 0xFE;
    sendCAN(); 
}



void DamiaoMotor::sendIdleFrame()
{
   
    tx_id_ = slave_id_;

    tx_data_[0] = 0x00; // pos
    tx_data_[1] = 0x00;
    tx_data_[2] = 0x00; // vel
    tx_data_[3] = 0x00;
    tx_data_[4] = 0x00; // kp
    tx_data_[5] = 0x00; // kd
    tx_data_[6] = 0x00; // torque
    tx_data_[7] = 0x00;

    sendCAN();
}

bool DamiaoMotor::getControlMode()
{
    readParam(0x0A);

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    updateState(MotorRefreshMode::Response);

    return true;
}

bool DamiaoMotor::setControlMode(DamiaoCtrlMode mode)
{

    ctrl_mode_ = mode;
    // 模式切换 RID=0x0A
    switch (mode)  
    {
        case DamiaoCtrlMode::MIT_MODE:
            writeParam(0x0A, (uint32_t)1);
            break;

        case DamiaoCtrlMode::POS_VEL_MODE:
            writeParam(0x0A, (uint32_t)2);
            break;

        case DamiaoCtrlMode::VEL_MODE:
            writeParam(0x0A, (uint32_t)3);
            break;

        case DamiaoCtrlMode::POS_FORCE_MODE:
            writeParam(0x0A, (uint32_t)4);  
            break;
    }

  
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    // 保存参数
    // saveParam();
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));
    updateState(MotorRefreshMode::Response);
    return true;
}


void DamiaoMotor::controlMIT(float pos, float vel, float tau, float kp, float kd)
{

    pos = std::clamp(pos, -limit_.Q_MAX, limit_.Q_MAX);
    vel = std::clamp(vel, -limit_.DQ_MAX, limit_.DQ_MAX);
    kp  = std::clamp(kp, 0.0f, 500.0f);
    kd  = std::clamp(kd, 0.0f, 5.0f);
    tau = std::clamp(tau, -limit_.TAU_MAX, limit_.TAU_MAX);

    uint16_t pos_uint = floatToUint(pos, -limit_.Q_MAX, limit_.Q_MAX, 16);
    uint16_t vel_uint = floatToUint(vel, -limit_.DQ_MAX, limit_.DQ_MAX, 12);
    uint16_t kp_uint  = floatToUint(kp, 0.0f, 500.0f, 12);
    uint16_t kd_uint  = floatToUint(kd, 0.0f, 5.0f,  12);
    uint16_t tau_uint = floatToUint(tau, -limit_.TAU_MAX, limit_.TAU_MAX, 12);

    tx_data_[0] = (pos_uint >> 8) & 0xFF;
    tx_data_[1] = pos_uint & 0xFF;
    tx_data_[2] = (vel_uint >> 4) & 0xFF;
    tx_data_[3] = ((vel_uint & 0x0F) << 4) | ((kp_uint >> 8) & 0x0F);
    tx_data_[4] = kp_uint & 0xFF;
    tx_data_[5] = (kd_uint >> 4) & 0xFF;
    tx_data_[6] = ((kd_uint & 0x0F) << 4) | ((tau_uint >> 8) & 0x0F);
    tx_data_[7] = tau_uint & 0xFF;

    tx_id_ = slave_id_;
    sendCAN(); 
}

void DamiaoMotor::controlPosVel(float pos, float vel)
{
    pos = std::clamp(pos, -limit_.Q_MAX, limit_.Q_MAX);
    vel = std::clamp(vel, -limit_.DQ_MAX, limit_.DQ_MAX);

    tx_id_ = 0x100 + slave_id_;
    memcpy(tx_data_, &pos, 4);
    memcpy(tx_data_ + 4, &vel, 4);
    sendCAN(); 
}


void DamiaoMotor::controlVel(float vel)
{
    vel = std::clamp(vel, -limit_.DQ_MAX, limit_.DQ_MAX);

    tx_id_ = 0x200 + slave_id_;
    memcpy(tx_data_, &vel, 4);
    sendCAN(); 
}


void DamiaoMotor::controlPositionTorque(float pos, float vel_limit, float current_limit)
{

    pos = std::clamp(pos, -limit_.Q_MAX, limit_.Q_MAX);
    vel_limit = std::clamp(vel_limit, 0.0f, limit_.DQ_MAX); // 速度限幅不能为负
    current_limit = std::clamp(current_limit, 0.0f, limit_.TAU_MAX);

    tx_id_ = 0x300 + slave_id_;  
    memcpy(tx_data_, &pos, 4);


    uint16_t v_lim = (uint16_t)(vel_limit * 100.0f);
    tx_data_[4] = (v_lim >> 8) & 0xFF;
    tx_data_[5] = v_lim & 0xFF;

  
    uint16_t i_lim = (uint16_t)(current_limit * 10000.0f);
    tx_data_[6] = (i_lim >> 8) & 0xFF;
    tx_data_[7] = i_lim & 0xFF;

    sendCAN(); 
}


void DamiaoMotor::readParam(uint8_t rid)
{
    tx_id_ = 0x7FF;

    memset(tx_data_, 0x00, 8);

    tx_data_[0] = slave_id_ & 0xFF;
    tx_data_[1] = (slave_id_ >> 8) & 0xFF;
    tx_data_[2] = 0x33;
    tx_data_[3] = rid;

    sendCAN();

}

//写 float 类型参数 (位置扭矩映射范围等)
void DamiaoMotor::writeParam(uint8_t rid, float value)
{
    tx_id_ = 0x7FF;
    memset(tx_data_, 0x00, 8);
    tx_data_[0] = slave_id_ & 0xFF;
    tx_data_[1] = (slave_id_ >> 8) & 0xFF;
    tx_data_[2] = 0x55;       // 写命令
    tx_data_[3] = rid;         // 寄存器
    memcpy(tx_data_ + 4, &value, 4);
    sendCAN();

}

//  写 UINT32 类型参数 (控制模式, 电机ID, 波特率等)
void DamiaoMotor::writeParam(uint8_t rid, uint32_t value)
{
    tx_id_ = 0x7FF;
    memset(tx_data_, 0x00, 8);
    tx_data_[0] = slave_id_ & 0xFF;
    tx_data_[1] = (slave_id_ >> 8) & 0xFF;
    tx_data_[2] = 0x55; // 写命令
    tx_data_[3] = rid;
    memcpy(tx_data_ + 4, &value, 4); // 拷贝 uint32
    sendCAN();
}


void DamiaoMotor::saveParam()
{
    tx_id_ = 0x7FF;
    memset(tx_data_, 0x00, 8);
    tx_data_[0] = slave_id_ & 0xFF;
    tx_data_[1] = (slave_id_ >> 8) & 0xFF;
    tx_data_[2] = 0xAA;       // 保存命令
    tx_data_[3] = 0x01;
    sendCAN();
}

bool DamiaoMotor::sendCAN()
{
    if (!m_can_bus) return false;
    return m_can_bus->send(tx_id_, tx_data_, 8);
    return true;
}

void DamiaoMotor::updateState(MotorRefreshMode refresh_mode )
{

    if (!m_can_bus) return;
    
    uint8_t data[8];
    uint8_t len;
    
    //状态反馈帧
    if(refresh_mode == MotorRefreshMode::Feedback)
    {

        bool is_online = m_can_bus->getFrame(master_id_, data, len);

        if (len == 8) 
        {
            parseFeedback(data, len);
        }
   
        if (is_online != last_motor_online_)
        {
            last_motor_online_ = is_online;  // 保存新状态
            
            if (is_online)
            {
                printf("\033[32m[INFO] 电机ID:%d 连接成功 \033[0m\n", static_cast<int>(master_id_ - 0x10));
            }
            else
            {
                printf("\033[31m[ERROR] 电机ID:%d 离线(超时) \033[0m\n", static_cast<int>(master_id_ - 0x10));
            }
        }
        
    }else if(refresh_mode == MotorRefreshMode::Response)//参数读取帧
    {

        m_can_bus->getFrame(slave_id_, data, len);
        
        if (len != 8)
            return;

        uint8_t func = data[2];
        switch (func)
        {
            case 0x33://读取返回值
                parseReadOrWirteParam(data,len);
                printf("✅ 电机ID：%d 读取参数成功\n", slave_id_);
                break;
            case 0x55://写入返回值
                parseReadOrWirteParam(data,len);
                printf("✅ 电机ID：%d 写入参数成功\n", slave_id_);
                break;
            case 0xAA://储存返回值
                printf("✅ 电机ID：%d 保存参数成功\n", slave_id_);
                break;
        }
        


    }else
    {
        printf("CAN通道:%d 电机ID: %d  未知：refresh_mode ✅\n",m_can_bus->getChannel(), slave_id_);
    }
    printMotorStateChange();
    
}

bool DamiaoMotor::parseFeedback(const uint8_t* data, uint8_t len)
{
    if (len != 8) return false;

    std::lock_guard<std::mutex> lock(motor_state_mutex_);

    // 第0字节：状态 + 错误码
    uint8_t status = data[0];
    motor_state_.state = static_cast<int>((status >> 4) & 0x0F);  // 电机状态
    motor_state_.motor_id = status & 0x0F;      // 返回帧电机id

    if(motor_state_.state == 1)
    {
        motor_state_.enabled = true;
    }
    else
    {
        motor_state_.enabled =false;
    }

    // 位置
    uint16_t pos_uint = (static_cast<uint16_t>(data[1]) << 8) | data[2];
    motor_state_.pos = uintToFloat(pos_uint, -limit_.Q_MAX, limit_.Q_MAX, 16);

    // 速度
    uint16_t vel_uint = (static_cast<uint16_t>(data[3]) << 4) | (data[4] >> 4);
    motor_state_.vel = uintToFloat(vel_uint, -limit_.DQ_MAX, limit_.DQ_MAX, 12);

    // 力矩
    uint16_t tau_uint = (static_cast<uint16_t>(data[4] & 0x0F) << 8) | data[5];
    motor_state_.tau = uintToFloat(tau_uint, -limit_.TAU_MAX, limit_.TAU_MAX, 12);

    // 温度
    motor_state_.temp_mos  = data[6];
    motor_state_.temp_coil = data[7];

    return true;
}

bool DamiaoMotor::parseReadOrWirteParam(const uint8_t* data, uint8_t len)
{
    if (len != 8)
        return false;
    uint8_t func_code = data[2];
    uint8_t reg_addr = data[3];

    uint32_t param_val = (static_cast<uint32_t>(data[4])      ) |
                    (static_cast<uint32_t>(data[5]) << 8 ) |
                    (static_cast<uint32_t>(data[6]) << 16) |
                    (static_cast<uint32_t>(data[7]) << 24);

    // 0x0A 控制模式寄存器 0x33读取 0x55写入
    if ( func_code == 0x33 && reg_addr == 0x0A)
    {
      
        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        switch (param_val)
        {   
            case 1:
                motor_state_.control_mode =1;
                printf("读取电机ID:%d MIT模式:%u\n", slave_id_, param_val);
                break;
            case 2:
                motor_state_.control_mode =2;
                printf("读取电机ID:%d 位置模式:%u\n", slave_id_, param_val);
                break;
            case 3:
                motor_state_.control_mode =3;
                printf("读取电机ID:%d 速度模式:%u\n", slave_id_, param_val);
                break;
            case 4:
                motor_state_.control_mode =4;
                printf("读取电机ID:%d 力位混控模式:%u\n", slave_id_, param_val);
                break;
            default:
                printf("⚠️ 读取电机ID:%d 未知模式:%u\n", slave_id_, param_val);
                return false;
        }
    }
    //写入控制模式返回解析
    else if (func_code == 0x55 && reg_addr == 0x0A)
    {        
        //printf("✅ 电机%d 写入参数成功 | 寄存器:0x%02X\n",slave_id_, reg_addr);

        std::lock_guard<std::mutex> lock(motor_state_mutex_);
        switch (param_val)
        {   
            case 1:
                motor_state_.control_mode =1;
                printf("写入电机ID:%d MIT模式:%u\n", slave_id_, param_val);
                break;
            case 2:
                motor_state_.control_mode =2;
                printf("写入电机ID:%d 位置模式:%u\n", slave_id_, param_val);
                break;
            case 3:
                motor_state_.control_mode =3;
                printf("写入电机ID:%d 速度模式:%u\n", slave_id_, param_val);
                break;
            case 4:
                motor_state_.control_mode =4;
                printf("写入电机ID:%d 力位混控模式:%u\n", slave_id_, param_val);
                break;
            default:
                printf("⚠️ 写入电机ID:%d 未知模式:%u\n", slave_id_, param_val);
                return false;
        }
    
    }

    return true;
}

void DamiaoMotor::printMotorStateChange()
{
    bool enable_changed = (motor_state_.enabled != motor_state_last_.enabled);
    bool state_changed  = (motor_state_.state != motor_state_last_.state);

    if (!enable_changed && !state_changed) {
        return;
    }

    std::string state_str;
    switch (motor_state_.state) {
        case 0:  state_str = "失能"; break;
        case 1:  state_str = "使能"; break;
        case 8:  state_str = "超压故障"; break;
        case 9:  state_str = "欠压故障"; break;
        case 10: state_str = "过流故障"; break;
        case 11: state_str = "MOS过温故障"; break;
        case 12: state_str = "线圈过温故障"; break;
        case 13: state_str = "通讯丢失"; break;
        case 14: state_str = "过载故障"; break;
        default: state_str = "未知状态"; break;
    }

    const char* green   = "\033[32m";    // 绿色
    const char* red     = "\033[31m";    // 红色
    const char* yellow  = "\033[33m";    // 黄色
    const char* reset   = "\033[0m";     // 复位

    const char* enable_color = motor_state_.enabled ? green : yellow;
    const char* enable_text  = motor_state_.enabled ? "ON" : "OFF";

    const char* state_color = (motor_state_.state == 1) ? green : red;

    std::cout << "[电机" << static_cast<int>(slave_id_) << "] "
         << "状态变化 → "
         << "使能: " << enable_color << enable_text << reset << " | "
         << "状态: " << state_color << state_str << reset
         << std::endl;

    motor_state_last_ = motor_state_;
}
