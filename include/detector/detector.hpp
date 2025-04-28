#pragma once
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <atomic>
namespace detector {
class detector {
public:
    detector();
    ~detector();
    
    void start_capture();
    void stop_capture();
    bool is_capturing() const;
    
    // 获取最新帧（线程安全）
    std::pair<std::shared_ptr<const cv::Mat>, uint64_t> get_latest_frame() const;

private:
    void main_loop();  

    cv::VideoCapture cap_;
    std::atomic<bool> capturing_{false};
    std::thread capture_thread_;
};
} // namespace detector