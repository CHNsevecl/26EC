#ifndef UART_HPP
#define UART_HPP

#include <string>
#include <vector>
#include <stdint.h>
#include <mutex>
#include <atomic>
#include <thread>
#include <chrono>

class UART {
private:
    int fd;
    
    // FIFO 相关成员
    static constexpr size_t FIFO_SIZE = 4096;   // 4KB FIFO缓冲区
    static constexpr size_t CLEAR_SIZE = 2048;  // 溢出时清除2KB
    std::vector<uint8_t> fifo_buffer;           // 环形缓冲区
    size_t head;                                // 读指针
    size_t tail;                                // 写指针
    std::mutex fifo_mutex;                      // 互斥锁保护FIFO
    
    std::atomic<bool> running;                  // 后台线程运行标志
    std::thread read_thread;                    // 后台读取线程
    
    // 溢出相关
    bool overflow_occurred;                     // 溢出标志
    size_t overflow_count;                      // 溢出计数器
    size_t max_fifo_usage;                      // FIFO最大使用量（用于监控）
    
    // 监控相关
    std::thread monitor_thread;
    std::atomic<bool> monitor_running;
    float warning_threshold;                    // 警告阈值 (0.0-1.0)
    float critical_threshold;                   // 严重阈值 (0.0-1.0)
    
    // 溢出策略枚举
    enum OverflowStrategy {
        CLEAR_OLDEST,        // 清除最旧的数据
        CLEAR_ALL,           // 清空整个FIFO
        DROP_NEWEST,         // 丢弃新数据
        AUTO_CLEAR           // 自动根据情况选择
    };
    OverflowStrategy overflow_strategy;
    
    // 内部函数
    void readLoop();
    void monitorLoop();
    void handleFIFOOverflow();
    size_t getFIFOUsage();
    void clearFIFOInternal();

public:
    // 构造函数和析构函数
    UART();
    ~UART();

    // 1. 初始化串口
    bool init(const char* port, int baudrate);

    // 2. 发送数据
    void send_string(const std::string& data);
    void send_uint8vector(const std::vector<uint8_t>& data);

    // 3. 获取FIFO中可用数据量
    size_t available();

    // 4. 从FIFO读取数据
    std::vector<uint8_t> read_from_fifo(size_t max_len);
    bool read_byte(uint8_t& byte);

    // 5. 清空FIFO
    void flush_rx();

    // 6. 从FIFO接收字符串（兼容原有接口）
    std::vector<std::string> receive_string();
    
    // 7. 接收指定格式的数据
    std::vector<uint8_t> receive_uint8_vector(uint8_t start_byte, size_t len=10, int timeout_ms=10);
    
    // 8. 获取FIFO状态（调试用）
    void print_fifo_status();
    float getFIFOUtilization();
    
    // 9. 溢出相关操作
    bool hasOverflow() const { return overflow_occurred; }
    size_t getOverflowCount() const { return overflow_count; }
    void clearOverflowFlag() { overflow_occurred = false; }
    void setOverflowStrategy(int strategy);  // 0:CLEAR_OLDEST, 1:CLEAR_ALL, 2:DROP_NEWEST, 3:AUTO_CLEAR
    
    // 10. 监控相关
    void setWarningThreshold(float threshold);
    void setCriticalThreshold(float threshold);
    void startMonitor();
    void stopMonitor();
    
    // 11. 关闭串口
    void close_port();

    // 12. 静态工具函数：16进制打印
    static std::string hex_dump(const uint8_t* data, size_t len, const std::string& prefix = "");
    static std::string hex_dump(const std::vector<uint8_t>& data, const std::string& prefix = "");
    static void print_hex(const std::vector<uint8_t>& data, const std::string& prefix = "");
    static void print_hex_with_ascii(const std::vector<uint8_t>& data, const std::string& prefix = "");

    // 禁止拷贝和赋值
    UART(const UART&) = delete;
    UART& operator=(const UART&) = delete;
};

#endif // UART_HPP