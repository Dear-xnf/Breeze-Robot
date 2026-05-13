#ifndef CAN_BUS_HPP
#define CAN_BUS_HPP

#include<iostream>
#include <cstdint>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>
#include <atomic>
#include <unordered_map>
#include "controlcan.h"
/**
 * @brief 创芯 USB-CAN 适配器封装类
 * @note 兼容周立功 VCI 标准接口（文档 V2.05）
 */
struct CanFrame
{
    uint8_t data[8] = {0};
    uint8_t len = 0;
    unsigned long last_update_ts = 0;  // <-- 只加这一行！
};
class CanBus
{
public:
    CanBus();
    ~CanBus();

    /**
     * @brief 打开 CAN 设备
     * @param dev_type  设备类型：4 = USBCAN2（文档推荐）
     * @param dev_idx   设备索引，默认 0 插入设备
     * @param ch        通道号，默认 0 第几个can口
     * @param baud      波特率，默认 1000000
     * @return 成功 true
     */
    bool open(  uint32_t ch = 0,
                uint32_t baud = 1000000,
                int32_t dev_idx = 0,
                uint32_t dev_type = 4);

    void close();

    /**
     * @brief 发送标准数据帧
     * @param id    CAN 报文 ID
     * @param data  数据指针
     * @param len   长度 1~8
     * @return 发送成功 true
     */
    bool send(uint32_t id, const uint8_t* data, uint8_t len = 8);

    /**
     * @brief 接收一帧报文
     * @param out_id    输出报文 ID
     * @param out_data  输出数据
     * @param out_len   输出长度
     * @return 成功=1，无数据=0，异常<0
     */

    // 从硬件读空所有帧 → 更新软件缓存
    void updateCache();

    // 按ID获取最新一帧（你要的“读取自己需要的帧”）
    bool getFrame(uint32_t id, uint8_t* out_data, uint8_t& out_len);
    uint64_t getCurrentMs();

    /// 获取设备是否已打开
    bool isOpened() const { return is_opened_; }

    uint32_t getChannel()const {return ch_;}

private:
    inline static bool s_is_opend_device = false;
    uint32_t dev_type_;   // 设备类型
    uint32_t dev_idx_;    // 设备索引
    uint32_t ch_;         // 通道
    bool     is_opened_;   // 打开标志

    std::thread read_thread_;
    std::atomic<bool> is_run_thread_ = false;
    std::mutex  frame_cache_mutex_;

    static const uint32_t MAX_READ_FRAMES = 2500;
    VCI_CAN_OBJ frame_buf_[MAX_READ_FRAMES]; //缓存can分析仪数据 
    std::unordered_map<uint32_t, CanFrame> frame_cache_; //按id储存最新单帧数据
};

#endif