#ifndef ZMQ_SEND_HPP
#define ZMQ_SEND_HPP

#include <opencv4/opencv2/opencv.hpp>
#include <zmq.hpp>
#include <thread>
#include <mutex>
#include <chrono>
#include <vector>

void zmq_sender(std::mutex& g_frame_mutex, cv::Mat& g_frame);
void update_frame(cv::Mat new_frame, cv::Mat& g_frame, std::mutex& g_frame_mutex);

#endif // ZMQ_SEND_HPP