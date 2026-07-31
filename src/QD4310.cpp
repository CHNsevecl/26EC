#include "QD4310.hpp"

/*!
 * \brief 初始化QD4310
 */
void QD4310::QD4310_Init() {
    // 设置溢出策略 (0:CLEAR_OLDEST, 1:CLEAR_ALL, 2:DROP_NEWEST, 3:AUTO_CLEAR)
    uart_qd4310.setOverflowStrategy(0);  // 删除最早2KB数据
    uart_qd4310.setCriticalThreshold(0.9);  // 90%时紧急清理
    if (!uart_qd4310.init(UART_ID,BAUD_RATE)){
        std::cout << UART_ID << " " << "初始化错误" << std::endl;
    }
}

std::vector<uint8_t> QD4310::QD4310_ReceiveData(uint8_t start, uint32_t timeout_ms){
    std::vector<uint8_t> data;
    data.reserve(10);
    bool start_flag = false;
    data = uart_qd4310.receive_uint8_vector(start, 10, timeout_ms);
    
    if(!data.empty()){
        if(data[0] == start ){
            return data;
        }
        else{
            std::cout << "数据帧格式错误"<< std::endl;
            return{};
        }
    }
    else{
        std::cout << "空数据"<<std::endl;
        return {};
    }
}

void QD4310::QD4310_SendData(const std::vector<uint8_t>& data){
    uart_qd4310.send_uint8vector(data);
}


/*!
 * \brief 控制QD4310
 * \param addr 地址
 * \param Control_mode 控制模式，详见 #QD4310_ControlMode_t
 * \param Control_quantity 控制量   
 * 
 */
void QD4310::QD4310_Contol(uint8_t addr ,uint8_t Control_mode ,uint16_t Control_quantity) {
    std::vector<uint8_t> command;
    command.reserve(5);
    command.push_back(addr); //校验码占位
    command.push_back(Control_mode); //控制模式
    command.push_back(Control_quantity & 0xFF); //命令字
    command.push_back(Control_quantity >> 8);
    command.push_back(CRC8(command)); //地址
    

    if(Control_mode == QD4310_MODE_REPORT){
        uart_qd4310.flush_rx();
        QD4310_SendData(command);
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); //等待电机响应
        std::vector<uint8_t> datas = QD4310_ReceiveData(addr);
        if(datas.size() == 10){
            uint8_t CRC8_Byte = datas[9];
            datas.pop_back();
            if(CRC8(datas) == CRC8_Byte){
                feedback.status = std::optional<uint8_t>(datas[1]);
                feedback.elc_current = std::optional<double>((int16_t)(datas[3] | (datas[4] << 8))/32767.0*10.0);
                feedback.speed = std::optional<uint16_t>((uint16_t)(datas[5] | (datas[6] << 8)));
                feedback.angle = std::optional<double>((double)(uint16_t)(datas[7] | (datas[8] << 8)) / 65535.0 * 360.0);
            }
            
        }else{
            feedback.status = std::nullopt;
            feedback.elc_current = std::nullopt;
            feedback.speed = std::nullopt;
            feedback.angle = std::nullopt;
        }
        
       
    }
    else{
        QD4310_SendData(command);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); //等待电机响应
}


/**
 * @brief 计算CRC-8校验码（多项式 x^8 + x^2 + x + 1）
 * @param data 数据向量，约定 data[0] 为CRC占位字节（不参与计算）
 * @return CRC-8校验码
 * @note 计算结果应存入 data[0]
 */
uint8_t QD4310::CRC8(const std::vector<uint8_t>& data) {
    uint8_t crc = 0x00;
    for (int i = 0; i < data.size(); ++i) {
        crc ^= data[i];
        for (int j = 0; j < 8; ++j) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07; // 多项式 x^8 + x^2 + x + 1
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t QD4310::rad (double angle){
    angle = std::max(angle_min,std::min(angle_max,angle));
    if(angle < 0){
        angle = 2*PI - std::abs(angle);
    }
    return static_cast<uint16_t>(angle /(2*PI)*65535);
}