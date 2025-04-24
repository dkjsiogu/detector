#pragma once
#include <opencv2/opencv.hpp>

namespace detector {

class detector {
public:
  detector();  // 构造函数声明
  cv::Mat share_image(const cv::Mat& img);
private:
  
};

} // namespace detector