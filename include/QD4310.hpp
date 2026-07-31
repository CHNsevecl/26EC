#ifndef QD4310_HPP
#define QD4310_HPP

//硬件库
#include "uart.hpp"
//向量库
#include <vector>
//字节库
#include <cstdint>
#include <iostream>
#include <optional>
//延时库
#include <chrono>
#include <thread>

#define angle_max 0.5
#define angle_min -0.5
#define UART_ID "/dev/ttyAMA10"
#define BAUD_RATE 115200
#define PI 3.14159265358979323846

/*!
 * \brief QD4310 控制模式枚举
 */
typedef enum {
    QD4310_MODE_REPORT    = 0x00, /*!< 报告模式 */
    QD4310_MODE_ENABLE    = 0x01, /*!< 电机使能 */
    QD4310_MODE_DISABLE   = 0x02, /*!< 电机失能 */
    QD4310_MODE_TORQUE    = 0x03,  /*!< 力矩控制模式 */
    QD4310_MODE_SPEED     = 0x04, /*!< 速度控制模式 */
    QD4310_MODE_ANGLE     = 0x05, /*!< 角度控制模式 */
    QD4310_MODE_LOWSPEED  = 0x06, /*!< 低速模式 */
    QD4310_MODE_STEPANGLE = 0x07  /*!< 角度步进控制 */
} QD4310_ControlMode_t;

struct QD4310_Feedback {
    std::optional<uint8_t> status;         /*!< 电机状态 */
    std::optional<double> elc_current;   /*!< 电机位置 */
    std::optional<uint16_t> speed;         /*!< 电机速度 */
    std::optional<double> angle;            /*!< 电机角度 */
};


class QD4310 {
private:
    UART uart_qd4310;
    void QD4310_SendData(const std::vector<uint8_t>& data);
    std::vector<uint8_t> QD4310_ReceiveData(uint8_t start, uint32_t timeout_ms = 10);
public:
    QD4310(){
        QD4310_Init();
    }
    QD4310_Feedback feedback;
    void QD4310_Init();
    void QD4310_Contol(uint8_t addr ,uint8_t Control_mode ,uint16_t Control_quantity);
    uint16_t rad (double angle);
    uint8_t CRC8(const std::vector<uint8_t>& data);
};

#endif