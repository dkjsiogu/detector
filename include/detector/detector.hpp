#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include "detector/inference.h"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <memory>

namespace detector {

    
    

class detector {
public:
    detector();
    ~detector();
    
    void start_capture();
    void stop_capture();
    bool is_capturing() const;
    const cv::Mat return_frame() const;

private:

    void capture_thread_func();
    
    cv::VideoCapture cap_;
    std::atomic<bool> capturing_{false};
    

    cv::dnn::Net net_arrow_;
    cv::dnn::Net net_cylida_;
    mutable std::mutex frame_mutex_;  // 用于保护current_frame_
    cv::Mat current_frame_;           // 存储当前帧

    std::unique_ptr<Inference> arrow_inference_;  // 声明箭头检测推理器
    std::unique_ptr<Inference> cylida_inference_; // 声明圆柱体检测推理器

    // thread
    std::thread capture_thread_;
    
};
} // namespace detector