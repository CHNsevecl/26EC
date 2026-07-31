#include "../include/zmq_send.hpp"

void update_frame(cv::Mat new_frame, cv::Mat& g_frame, std::mutex& g_frame_mutex) {
    std::lock_guard<std::mutex> lock(g_frame_mutex);
    new_frame.copyTo(g_frame);
}

void zmq_sender(std::mutex& g_frame_mutex, cv::Mat& g_frame){
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUSH);
    socket.bind("tcp://*:5555");
    
    std::cout << "ZMQ Sender started on tcp://*:5555" << std::endl;
    
    while (true) {
        cv::Mat frame_copy;
        {
            std::lock_guard<std::mutex> lock(g_frame_mutex);
            if (g_frame.empty()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                continue;
            }
            g_frame.copyTo(frame_copy); // 拷贝一份，避免长时间持锁
        }
        
        // 编码为JPEG（压缩后传输，节省带宽）
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        cv::imencode(".jpg", frame_copy, buffer, params);
        
        // 通过ZMQ发送
        zmq::message_t message(buffer.data(), buffer.size());
        socket.send(message, zmq::send_flags::none);
        
        // 控制发送帧率，避免占用过多CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(33)); // ~30fps
    }
}