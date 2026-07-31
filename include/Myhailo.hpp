#ifndef __MYHAILO_HPP__
#define __MYHAILO_HPP__

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include "opencv4/opencv2/opencv.hpp"
#include <chrono>
#include <hailo/hailort.hpp>

const int target_w = 640;
const int target_h = 640;
const std::vector<std::string> class_names = {"steel_ball","pipe"};
const std::string hef_path = "/home/sevecl/Desktop/C/Example/26电赛/yolov8s.hef";
constexpr size_t expected_input = 640*640*3;
const int max_boxes_per_class = 100; // 每个类别最多100个框
const float score_threshold = 0.75f; // 画框阈值
 
struct HailoContext {
    std::shared_ptr<hailort::VDevice> vdevice;
    std::shared_ptr<hailort::InferModel> infer_model;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;
    std::vector<hailort::Buffer> input_buffer;  
    std::vector<hailort::Buffer> output_buffer;  
    hailort::ConfiguredInferModel configured_infer_model;
    hailort::ConfiguredInferModel::Bindings bindings;
};

struct Detection {
    std::string label;
    cv::Point upper;
    cv::Point lower;
    float score;
};

std::optional<HailoContext> Hailo_init(const std::string& hef_path);
std::optional<std::vector<Detection>> ParseDetections(HailoContext& hailo_context, int target_w, int target_h, cv::Mat& RGB_frame,const std::vector<std::string>& class_names);
cv::Mat Stream_process(const cv::Mat& BGR_frame, int target_w, int target_h);
cv::Mat Stream_resize(cv::Mat BGR_frame);

#endif