#pragma once
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unordered_map>
#include <memory>

namespace detector {

    enum ArrowDirection { UP, DOWN, LEFT, RIGHT, UNKNOWN };

class detector {
public:
    detector();
    ~detector();
    
    void start_capture();
    void stop_capture();
    bool is_capturing() const;
    
    std::shared_ptr<const cv::Mat> return_frame();

    std::shared_ptr<const cv::Mat> get_img(const std::string& name) const;
    
private:
    void init_img_map();
    void set_img(const std::string& name, cv::Mat&& img);
    
    void capture_thread_func();
    void process_thread_func();
    void display_thread_func();
    
    std::vector<cv::Point> findArrowContour(const cv::Mat& binary_mask);
    ArrowDirection determineDirection(const std::vector<cv::Point>& contour);
    
    cv::VideoCapture cap_;
    std::atomic<bool> capturing_{false};
    
    // thread
    std::thread capture_thread_;
    std::thread process_thread_;
    std::thread display_thread_;
    
    //lock
    mutable std::mutex img_mutex_;
    std::condition_variable frame_ready_;
    bool frame_processed_ = true;
    
    // img
    std::unordered_map<std::string, std::shared_ptr<cv::Mat>> img_map_;
    
    // HSV
    const cv::Scalar lower_green_ = cv::Scalar(35, 50, 50);
    const cv::Scalar upper_green_ = cv::Scalar(85, 255, 255);
};
} // namespace detector