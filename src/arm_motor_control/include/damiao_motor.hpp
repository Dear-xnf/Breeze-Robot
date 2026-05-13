#ifndef DAMIAO_MOTOR_HPP
#define DAMIAO_MOTOR_HPP

#include <cstdint>
#include <string>
#include <memory>
#include <mutex>
#include "can_bus.hpp"


enum DM_Motor_Type
{
    DM4310,
    DM4310_48V,
    DM4340,
    DM4340_48V,
    DM6006,
    DM8006,
    DM8009,
    DM10010L,
    DM10010,
    DMH3510,
    DMH6215,
    DMG6220,
    Num_Of_Motor
};

enum class DamiaoCtrlMode
{
    MIT_MODE      = 1,
    POS_VEL_MODE  = 2,
    VEL_MODE      = 3,
    POS_FORCE_MODE= 4,
};

struct MotorLimit
{
    float Q_MAX;   
    float DQ_MAX;   
    float TAU_MAX;  
};


struct MotorState {
    bool     enabled = 0;
    int      state = 0;
    int      control_mode = 0;
    uint8_t  motor_id = 0;
    float    pos = 0;
    float    vel = 0;
    float    tau = 0;
    float    temp_mos = 0;
    float    temp_coil = 0;

//  0  —— 失能
//  1  —— 使能
//  8  —— 超压
//  9  —— 欠压
//  A(10) —— 过电流
//  B(11) —— MOS 过温
//  C(12) —— 电机线圈过温
//  D(13) —— 通讯丢失
//  E(14) —— 过载
};

enum class MotorRefreshMode
{
    Feedback,
    Response,
    ControlMode,
    All
   
};


class DamiaoMotor
{
public:
    /**
     * @brief 构造函数
     * @param type      电机型号 DM4310...
     * @param slave_id  电机ID（发指令用）
     * @param master_id 主机ID（收反馈用）
     */
    DamiaoMotor() = default;
    DamiaoMotor( std::shared_ptr<CanBus> can_bus, DM_Motor_Type type, uint8_t slave_id, uint8_t master_id = 0x00);

    void enable();
    void disable();        
    void setZeroPosition(); 

    bool getControlMode();
    bool setControlMode(DamiaoCtrlMode mode);

    void sendIdleFrame();
    void controlMIT(float pos, float vel, float tau, float kp, float kd);
    void controlPosVel(float pos, float vel);
    void controlVel(float vel);
    void controlPositionTorque(float pos, float vel_limit, float current_limit);

    //读取数据前必须调用
    void updateState(MotorRefreshMode refresh_mode =MotorRefreshMode::Feedback);

    MotorState getMotorState() {std::lock_guard<std::mutex> lock(motor_state_mutex_); return motor_state_; }
    uint8_t getSlaveId() const { return slave_id_; }
    uint8_t getMasterId() const { return master_id_; }
    int getCanChannel() const { return static_cast<int>(m_can_bus->getChannel());}

private:
    uint16_t floatToUint(float x, float min, float max, uint8_t bits);
    float uintToFloat(uint16_t x, float min, float max, uint8_t bits);
    void loadMotorLimit(DM_Motor_Type type);

    bool parseFeedback(const uint8_t* data, uint8_t len);
    bool parseReadOrWirteParam(const uint8_t* data, uint8_t len);

    void readParam(uint8_t rid);
    void writeParam(uint8_t rid, float value);
    void writeParam(uint8_t rid, uint32_t talue);
    void saveParam();

    bool sendCAN();
    void printMotorStateChange();

private:
    uint8_t     slave_id_;      
    uint8_t     master_id_;    
    MotorLimit  limit_;        
    DamiaoCtrlMode ctrl_mode_;  

    
    MotorState motor_state_;
    MotorState motor_state_last_;
    std::mutex motor_state_mutex_;

    // 发送缓存
    uint32_t tx_id_;
    uint8_t  tx_data_[8] = {0};

    std::shared_ptr<CanBus> m_can_bus;

    bool last_motor_online_ = false; 
};

#endif // DAMIAO_MOTOR_HPP