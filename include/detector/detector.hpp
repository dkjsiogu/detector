#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <memory>

namespace detector {

    enum ArrowDirection { UNKNOWN,UP, LEFT, RIGHT};

class detector {
public:
    detector();
    ~detector();
    
    void start_capture();
    void stop_capture();
    bool is_capturing() const;

    std::string get_result(ArrowDirection Dir);
    
private:

    void capture_thread_func();
    
    cv::VideoCapture cap_;
    std::atomic<bool> capturing_{false};
    ArrowDirection result;
    const std::vector<std::string> class_names_{"U", "R", "L"};
    cv::dnn::Net net_arrow_;
    cv::dnn::Net net_cylida_;
    // thread
    std::thread capture_thread_;
    
};
} // namespace detector