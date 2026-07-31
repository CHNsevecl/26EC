#include "../include/uart.hpp"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <ctime>

using namespace std;

// ============ 构造函数和析构函数 ============

UART::UART() 
    : fd(-1), 
      fifo_buffer(FIFO_SIZE), 
      head(0), 
      tail(0), 
      running(false),
      overflow_occurred(false),
      overflow_count(0),
      max_fifo_usage(0),
      monitor_running(false),
      warning_threshold(0.7f),
      critical_threshold(0.9f),
      overflow_strategy(AUTO_CLEAR) {
}

UART::~UART() {
    close_port();
}

// ============ 内部函数 ============

size_t UART::getFIFOUsage() {
    // 注意：调用此函数前需要已持有锁
    if (tail >= head) {
        return tail - head;
    } else {
        return FIFO_SIZE - head + tail;
    }
}

void UART::clearFIFOInternal() {
    // 注意：调用此函数前需要已持有锁
    head = 0;
    tail = 0;
}

void UART::handleFIFOOverflow() {
    std::lock_guard<std::mutex> lock(fifo_mutex);
    
    overflow_occurred = true;
    overflow_count++;
    
    size_t current_usage = getFIFOUsage();
    
    cerr << "\n========================================" << endl;
    cerr << "⚠️ FIFO溢出发生 (第 " << overflow_count << " 次)" << endl;
    time_t now = time(nullptr);
    cerr << "   时间: " << ctime(&now);
    cerr << "   FIFO使用量: " << current_usage << "/" << FIFO_SIZE << " 字节" << endl;
    
    // 显示将要删除的数据（前16字节）
    const size_t DISPLAY_SIZE = 16;
    if (current_usage > 0) {
        cerr << "   将要删除的数据(前16字节): ";
        size_t temp_head = head;
        for (size_t i = 0; i < std::min(DISPLAY_SIZE, current_usage); ++i) {
            cerr << hex << setw(2) << setfill('0') 
                 << (int)fifo_buffer[temp_head] << " ";
            temp_head = (temp_head + 1) % FIFO_SIZE;
        }
        cerr << dec << endl;
    }
    
    // 根据策略处理溢出
    switch (overflow_strategy) {
        case CLEAR_OLDEST: {
            // 删除最早出现的2KB数据
            size_t to_clear = std::min(CLEAR_SIZE, current_usage);
            cerr << "   策略: 删除最早 " << to_clear << " 字节" << endl;
            for (size_t i = 0; i < to_clear; ++i) {
                head = (head + 1) % FIFO_SIZE;
            }
            break;
        }
        
        case CLEAR_ALL: {
            // 清空整个FIFO
            cerr << "   策略: 清空整个FIFO" << endl;
            clearFIFOInternal();
            break;
        }
        
        case AUTO_CLEAR: {
            // 根据FIFO使用率自动选择策略
            float usage = (float)current_usage / FIFO_SIZE;
            if (usage > 0.95) {
                // 使用率>95%，清空一半
                size_t to_clear = std::min(FIFO_SIZE / 2, current_usage);
                cerr << "   策略: 自动清理 - 清除 " << to_clear << " 字节" << endl;
                for (size_t i = 0; i < to_clear; ++i) {
                    head = (head + 1) % FIFO_SIZE;
                }
            } else {
                // 使用率适中，清除2KB
                size_t to_clear = std::min(CLEAR_SIZE, current_usage);
                cerr << "   策略: 自动清理 - 清除 " << to_clear << " 字节" << endl;
                for (size_t i = 0; i < to_clear; ++i) {
                    head = (head + 1) % FIFO_SIZE;
                }
            }
            break;
        }
        
        case DROP_NEWEST:
        default: {
            // 丢弃新数据（当前行为）
            cerr << "   策略: 丢弃新数据" << endl;
            break;
        }
    }
    
    // 显示清理后的状态
    size_t remaining = getFIFOUsage();
    cerr << "   清理后剩余: " << remaining << " 字节" << endl;
    
    // 如果清理后仍然接近满，继续清理（防止连续溢出）
    if (remaining > FIFO_SIZE * 0.8 && overflow_strategy != DROP_NEWEST) {
        cerr << "   ⚠️ 清理后使用率仍较高 (" << (remaining * 100 / FIFO_SIZE) << "%)" << endl;
        size_t extra_clear = std::min((size_t)1024, remaining);
        cerr << "   额外清理: 删除 " << extra_clear << " 字节" << endl;
        
        // 显示额外删除的数据
        size_t temp_head = head;
        cerr << "   额外删除的数据(前8字节): ";
        for (size_t i = 0; i < std::min((size_t)8, extra_clear); ++i) {
            cerr << hex << setw(2) << setfill('0') 
                 << (int)fifo_buffer[temp_head] << " ";
            temp_head = (temp_head + 1) % FIFO_SIZE;
        }
        cerr << dec << endl;
        
        for (size_t i = 0; i < extra_clear; ++i) {
            head = (head + 1) % FIFO_SIZE;
        }
        remaining = getFIFOUsage();
        cerr << "   额外清理后剩余: " << remaining << " 字节" << endl;
    }
    
    // 更新最大使用量
    if (current_usage > max_fifo_usage) {
        max_fifo_usage = current_usage;
    }
    
    cerr << "========================================\n" << endl;
}

void UART::readLoop() {
    uint8_t temp_buf[256];
    fd_set readfds;
    struct timeval tv;
    
    while (running.load()) {
        FD_ZERO(&readfds);
        FD_SET(fd, &readfds);
        
        tv.tv_sec = 0;
        tv.tv_usec = 10000; // 10ms超时，及时响应退出
        
        int ret = select(fd + 1, &readfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (running.load()) {
                perror("select error in UART read thread");
            }
            break;
        } else if (ret == 0) {
            continue; // 超时，继续循环
        }
        
        if (FD_ISSET(fd, &readfds)) {
            ssize_t n = read(fd, temp_buf, sizeof(temp_buf));
            if (n > 0) {
                std::lock_guard<std::mutex> lock(fifo_mutex);
                
                for (ssize_t i = 0; i < n; ++i) {
                    size_t next_tail = (tail + 1) % FIFO_SIZE;
                    if (next_tail == head) {
                        // FIFO溢出，调用处理函数
                        handleFIFOOverflow();
                        
                        // 重新检查是否还有空间
                        if ((tail + 1) % FIFO_SIZE == head) {
                            // 仍然没有空间，丢弃这个字节
                            cerr << "   ⚠️ 清理后仍无空间，丢弃字节: 0x" 
                                 << hex << (int)temp_buf[i] << dec << endl;
                            continue;
                        }
                        next_tail = (tail + 1) % FIFO_SIZE;
                    }
                    fifo_buffer[tail] = temp_buf[i];
                    tail = next_tail;
                }
            } else if (n < 0) {
                if (running.load()) {
                    perror("UART read error");
                }
                break;
            }
        }
    }
}

void UART::monitorLoop() {
    while (monitor_running.load()) {
        float usage = getFIFOUtilization();
        
        if (usage > critical_threshold) {
            cerr << "🚨 严重警告: FIFO使用率 " << (usage * 100) << "%" << endl;
            cerr << "   执行紧急清理..." << endl;
            
            // 紧急清理：清空一半数据
            std::lock_guard<std::mutex> lock(fifo_mutex);
            size_t current_usage = getFIFOUsage();
            size_t to_clear = current_usage / 2;
            for (size_t i = 0; i < to_clear && head != tail; ++i) {
                head = (head + 1) % FIFO_SIZE;
            }
            
        } else if (usage > warning_threshold) {
            cout << "⚠️ 警告: FIFO使用率 " << (usage * 100) << "%" << endl;
            cout << "   当前使用量: " << available() << "/" << FIFO_SIZE << " 字节" << endl;
        }
        
        // 每秒检查一次
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ============ 公共函数 ============

bool UART::init(const char* port, int baudrate) {
    // 打开设备，使用非阻塞方式
    fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        std::cout << "无法打开串口: " << port << std::endl;
        return false;
    }

    // 配置参数
    struct termios options;
    memset(&options, 0, sizeof(options));
    
    if (tcgetattr(fd, &options) != 0) {
        std::cout << "获取串口配置失败" << std::endl;
        return false;
    }

    // 设置波特率
    switch (baudrate) {
        case 9600: cfsetispeed(&options, B9600); cfsetospeed(&options, B9600); break;
        case 19200: cfsetispeed(&options, B19200); cfsetospeed(&options, B19200); break;
        case 38400: cfsetispeed(&options, B38400); cfsetospeed(&options, B38400); break;
        case 115200: cfsetispeed(&options, B115200); cfsetospeed(&options, B115200); break;
        default:
            std::cout << "不支持的波特率: " << baudrate << std::endl;
            return false;
    }
    
    // 控制选项
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // 本地选项
    options.c_lflag &= ~(ICANON | ECHO);
    options.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控

    // 设置超时
    options.c_cc[VMIN] = 0;    // 非阻塞读取
    options.c_cc[VTIME] = 0;   // 不等待超时

    // 应用配置
    tcsetattr(fd, TCSANOW, &options);
    
    // 清空硬件缓冲区
    tcflush(fd, TCIOFLUSH);
    
    // 清空软件FIFO
    {
        std::lock_guard<std::mutex> lock(fifo_mutex);
        clearFIFOInternal();
        overflow_occurred = false;
    }
    
    // 启动后台读取线程
    running.store(true);
    if (read_thread.joinable()) {
        read_thread.join();
    }
    read_thread = std::thread(&UART::readLoop, this);
    
    std::cout << "串口 " << port << " 初始化成功！" << std::endl;
    return true;
}

void UART::send_string(const std::string& data) {
    if (fd != -1) {
        write(fd, data.c_str(), data.length());
    }
}

void UART::send_uint8vector(const std::vector<uint8_t>& data) {
    if (fd != -1 && !data.empty()) {
        ssize_t result = write(fd, data.data(), data.size());
        if (result == -1) {
            perror("write failed");
        }
    }
}

size_t UART::available() {
    std::lock_guard<std::mutex> lock(fifo_mutex);
    return getFIFOUsage();
}

std::vector<uint8_t> UART::read_from_fifo(size_t max_len) {
    std::vector<uint8_t> result;
    std::lock_guard<std::mutex> lock(fifo_mutex);
    
    size_t avail = getFIFOUsage();
    size_t len = std::min(max_len, avail);
    result.reserve(len);
    
    for (size_t i = 0; i < len; ++i) {
        result.push_back(fifo_buffer[head]);
        head = (head + 1) % FIFO_SIZE;
    }
    
    return result;
}

bool UART::read_byte(uint8_t& byte) {
    std::lock_guard<std::mutex> lock(fifo_mutex);
    if (head == tail) {
        return false; // FIFO为空
    }
    byte = fifo_buffer[head];
    head = (head + 1) % FIFO_SIZE;
    return true;
}

void UART::flush_rx() {
    // 清空软件FIFO
    std::lock_guard<std::mutex> lock(fifo_mutex);
    clearFIFOInternal();
    overflow_occurred = false;
    
    // 清空硬件缓冲区
    if (fd != -1) {
        tcflush(fd, TCIFLUSH);
    }
}

std::vector<std::string> UART::receive_string() {
    std::vector<std::string> result;
    uint8_t byte;
    std::string line;
    
    // 从FIFO读取数据直到遇到换行符或超时
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count() < 10) {
        if (read_byte(byte)) {
            if (byte == '\n' || byte == '\r') {
                if (!line.empty()) {
                    result.push_back(line);
                    line.clear();
                }
                if (byte == '\r' && available() > 0) {
                    // 可能还有数据，继续读取
                    uint8_t next_byte;
                    if (read_byte(next_byte) && next_byte != '\n') {
                        // 如果不是标准的\r\n，把数据放回去
                        // 这里简化处理，重新构建
                    }
                }
            } else {
                line += static_cast<char>(byte);
            }
        } else {
            // FIFO为空，稍微等待
            usleep(100);
        }
    }
    
    // 如果最后还有未结束的行，也添加进去
    if (!line.empty()) {
        result.push_back(line);
    }
    
    return result;
}

std::vector<uint8_t> UART::receive_uint8_vector(uint8_t start_byte, size_t len, int timeout_ms) {
    std::vector<uint8_t> result;
    bool found_start = false;
    auto start_time = std::chrono::steady_clock::now();
    
    while (std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count() < timeout_ms) {
        
        uint8_t byte;
        if (read_byte(byte)) {
            if (!found_start) {
                if (byte == start_byte) {
                    found_start = true;
                    result.push_back(byte);
                }
            } else {
                result.push_back(byte);
                if (result.size() >= len) {
                    break;
                }
            }
        } else {
            usleep(100); // 避免忙等待
        }
    }
    
    return result;
}

void UART::print_fifo_status() {
    std::lock_guard<std::mutex> lock(fifo_mutex);
    size_t usage = getFIFOUsage();
    std::cout << "========== FIFO状态 ==========" << std::endl;
    std::cout << "head: " << head << std::endl;
    std::cout << "tail: " << tail << std::endl;
    std::cout << "可用数据: " << usage << " 字节" << std::endl;
    std::cout << "总大小: " << FIFO_SIZE << " 字节" << std::endl;
    std::cout << "使用率: " << (usage * 100.0 / FIFO_SIZE) << "%" << std::endl;
    std::cout << "最大使用量: " << max_fifo_usage << " 字节" << std::endl;
    std::cout << "溢出次数: " << overflow_count << std::endl;
    std::cout << "================================" << std::endl;
}

float UART::getFIFOUtilization() {
    std::lock_guard<std::mutex> lock(fifo_mutex);
    return (float)getFIFOUsage() / FIFO_SIZE;
}

void UART::setOverflowStrategy(int strategy) {
    switch (strategy) {
        case 0: overflow_strategy = CLEAR_OLDEST; break;
        case 1: overflow_strategy = CLEAR_ALL; break;
        case 2: overflow_strategy = DROP_NEWEST; break;
        case 3: overflow_strategy = AUTO_CLEAR; break;
        default: 
            std::cerr << "未知策略，使用默认 AUTO_CLEAR" << std::endl;
            overflow_strategy = AUTO_CLEAR;
    }
    std::cout << "溢出策略已设置为: " << strategy << std::endl;
}

void UART::setWarningThreshold(float threshold) {
    if (threshold > 0 && threshold < 1) {
        warning_threshold = threshold;
    }
}

void UART::setCriticalThreshold(float threshold) {
    if (threshold > 0 && threshold < 1) {
        critical_threshold = threshold;
    }
}

void UART::startMonitor() {
    if (!monitor_running.load()) {
        monitor_running.store(true);
        if (monitor_thread.joinable()) {
            monitor_thread.join();
        }
        monitor_thread = std::thread(&UART::monitorLoop, this);
        std::cout << "FIFO监控已启动" << std::endl;
    }
}

void UART::stopMonitor() {
    monitor_running.store(false);
    if (monitor_thread.joinable()) {
        monitor_thread.join();
        std::cout << "FIFO监控已停止" << std::endl;
    }
}

void UART::close_port() {
    if (fd != -1) {
        // 停止后台线程
        running.store(false);
        if (read_thread.joinable()) {
            read_thread.join();
        }
        
        stopMonitor();
        
        ::close(fd);
        fd = -1;
    }
}

// ============ 静态工具函数 ============

std::string UART::hex_dump(const uint8_t* data, size_t len, const std::string& prefix) {
    std::string result = prefix;
    char buffer[8];
    
    for (size_t i = 0; i < len; ++i) {
        if (i > 0 && i % 16 == 0) {
            result += "\n" + prefix;
        }
        snprintf(buffer, sizeof(buffer), "%02X ", data[i]);
        result += buffer;
    }
    
    return result;
}

std::string UART::hex_dump(const std::vector<uint8_t>& data, const std::string& prefix) {
    return hex_dump(data.data(), data.size(), prefix);
}

void UART::print_hex(const std::vector<uint8_t>& data, const std::string& prefix) {
    if (data.empty()) {
        std::cout << prefix << "(空数据)" << std::endl;
        return;
    }
    
    std::cout << prefix << "[" << data.size() << "字节] ";
    for (size_t i = 0; i < data.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)data[i] << " ";
        if ((i + 1) % 16 == 0 && i < data.size() - 1) {
            std::cout << std::endl << prefix << "         ";
        }
    }
    std::cout << std::dec << std::endl;
}

void UART::print_hex_with_ascii(const std::vector<uint8_t>& data, const std::string& prefix) {
    if (data.empty()) {
        std::cout << prefix << "(空数据)" << std::endl;
        return;
    }
    
    // 打印16进制
    std::cout << prefix << "HEX: ";
    for (size_t i = 0; i < data.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)data[i] << " ";
        if ((i + 1) % 16 == 0 && i < data.size() - 1) {
            std::cout << std::endl << prefix << "     ";
        }
    }
    std::cout << std::dec << std::endl;
    
    // 打印ASCII
    std::cout << prefix << "ASCII: ";
    for (auto b : data) {
        if (b >= 32 && b <= 126) {
            std::cout << (char)b;
        } else {
            std::cout << ".";
        }
    }
    std::cout << std::endl;
}