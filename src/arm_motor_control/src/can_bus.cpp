#include "can_bus.hpp"
#include <cstring>
#include <stdio.h>
#include <iomanip>  
#include <chrono>
#include <pthread.h>   

CanBus::CanBus()
    : dev_type_(0)
    , dev_idx_(0)
    , ch_(0)
    , is_opened_(false)
{
}

CanBus::~CanBus()
{
    close();
}
bool CanBus::open(  uint32_t ch ,uint32_t baud ,int32_t dev_idx,uint32_t dev_type)
{
    if (is_opened_) return true;

    ch_       = ch;
    dev_idx_  = dev_idx;
    dev_type_ = dev_type;

    if(s_is_opend_device ==false)
    {
        if (VCI_OpenDevice(dev_type_, dev_idx_, 0) != 1) {
            printf("dev_idx_：%d  VCI_OpenDevice 失败\n",dev_idx_);
            return false;
        }
        s_is_opend_device = true;

    }
   
    VCI_INIT_CONFIG cfg{};
    cfg.AccCode   = 0x00000000;
    cfg.AccMask   = 0xFFFFFFFF;
    cfg.Filter    = 0;      // 接收所有帧
    cfg.Mode      = 0;      // 正常模式

    switch (baud) {
        case 1000000: cfg.Timing0 = 0x00; cfg.Timing1 = 0x14; break;
        case 500000:  cfg.Timing0 = 0x00; cfg.Timing1 = 0x1C; break;
        case 250000:  cfg.Timing0 = 0x01; cfg.Timing1 = 0x1C; break;
        case 125000:  cfg.Timing0 = 0x03; cfg.Timing1 = 0x1C; break;
        default:      cfg.Timing0 = 0x00; cfg.Timing1 = 0x14; break;
    }

    if (VCI_InitCAN(dev_type_, dev_idx_, ch_, &cfg) != 1) {
        printf("CAN分析仪通道: CAN%d  VCI_InitCAN 失败\n",static_cast<int>(ch_+1));
        VCI_CloseDevice(dev_type_, dev_idx_);
        return false;
    }

 
    if (VCI_StartCAN(dev_type_, dev_idx_, ch_) != 1) {
        printf("CAN分析仪通道: CAN%d  VCI_StartCAN 失败\n",static_cast<int>(ch_+1));
        VCI_CloseDevice(dev_type_, dev_idx_);
        return false;
    }


    is_opened_ = true;

    printf("CAN分析仪通道: CAN%d 打开成功！ ✅ \n",(ch_+1));

    is_run_thread_ = true;
    pthread_setname_np(pthread_self(), "CAN_Read_Thread");
    read_thread_ = std::thread([this]() 
    {
    while (is_run_thread_) {updateCache();std::this_thread::sleep_for(std::chrono::milliseconds(5));}
    });

    return true;
}

void CanBus::close()
{
    if (is_run_thread_) {
        is_run_thread_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
    }

    if (is_opened_) {
        VCI_CloseDevice(dev_type_, dev_idx_);
        is_opened_ = false;
        printf("CAN 已关闭\n");
    }
}

bool CanBus::send(uint32_t id, const uint8_t* data, uint8_t len)
{
    if (!is_opened_ || !data || len > 8)
        return false;

    VCI_CAN_OBJ frame{};
    frame.ID          = id;
    frame.SendType    = 0;        // 正常发送
    frame.RemoteFlag  = 0;        // 数据帧
    frame.ExternFlag  = 0;        // 标准帧
    frame.DataLen     = len;
    memcpy(frame.Data, data, len);

    return VCI_Transmit(dev_type_, dev_idx_, ch_, &frame, 1) == 1;
}

void CanBus::updateCache()
{

    if (!is_opened_)
    return;


    int frame_cnt = VCI_Receive(
        dev_type_,
        dev_idx_,
        ch_,
        frame_buf_,        
        MAX_READ_FRAMES,
        0
    );

    if (frame_cnt <= 0)
        return;

    // 处理所有帧
    std::lock_guard<std::mutex> lock(frame_cache_mutex_);

    for (int i = 0; i < frame_cnt; i++)
    {
        VCI_CAN_OBJ& frame = frame_buf_[i];

        if (frame.DataLen > 8)
            continue;

        CanFrame f;
        memcpy(f.data, frame.Data, frame.DataLen);
        f.len = frame.DataLen;
        f.last_update_ts = getCurrentMs();

        uint32_t cache_id;

        // 判断：【参数响应帧】（读/写/保存参数）
        uint8_t func = frame.Data[2];
        if (func == 0x33 || func == 0x55 || func == 0xAA)//返回参数，读取，写入，存储
        {
            uint16_t motor_id = frame.Data[0] | ( (uint16_t)frame.Data[1] << 8 );
            cache_id = motor_id;
        }
        else
        {
            cache_id = frame.ID;
        }

        // 保存到 map
        frame_cache_[cache_id] = f;


        //=========================打印接收到的报文=============================
        // std::cout <<"CAN"<<(ch_+1) <<" ID=0x" << std::hex << frame.ID << std::dec 
        //   << " data=" 
        //   << std::hex << (int)f.data[0] << " " << std::hex << (int)f.data[1] << " " 
        //   << std::hex << (int)f.data[2] << " " << std::hex << (int)f.data[3] << " " 
        //   << std::hex << (int)f.data[4] << " " << std::hex << (int)f.data[5] << " " 
        //   << std::hex << (int)f.data[6] << " " << std::hex << (int)f.data[7] << "\n";


        //=========================打印接收到的报文=============================
        // std::stringstream ss;
        // ss << "CAN" << (ch_ + 1)
        // << " ID=0x" << std::uppercase << std::setw(3) << std::setfill('0')
        // << std::hex << frame.ID << std::nouppercase << std::dec
        // << " data=";

        // for (int i = 0; i < 8; ++i) {
        //     ss << std::uppercase << std::hex << std::setw(2) << std::setfill('0')
        //     << static_cast<int>(f.data[i]) << std::nouppercase << " ";
        // }

        // // 一次性打印（原子操作，不会被打断）
        // std::cout << ss.str() << "\n";


    }

}


bool CanBus::getFrame(uint32_t id, uint8_t* out_data, uint8_t& out_len)
{
    if (!out_data)
        return false;

    std::lock_guard<std::mutex> lock(frame_cache_mutex_);

    auto it = frame_cache_.find(id);
    if (it == frame_cache_.end())
        return false;

    memcpy(out_data, it->second.data, it->second.len);
    out_len = it->second.len;

    uint64_t now = getCurrentMs();
    if (now - it->second.last_update_ts > 500)
    {
        return false; // 离线返回
    }

    return true;
}

uint64_t CanBus::getCurrentMs()
{
    auto now = std::chrono::system_clock::now();
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch());
    return ms.count();
}