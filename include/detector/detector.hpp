#pragma once
#include <opencv2/opencv.hpp>
#include <memory>
#include <mutex>          
#include <unordered_map>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
namespace detector {

class detector {
public:
  detector();  // 构造函数声明
  std::shared_ptr<const cv::Mat> get_img(const std::string& name) const;
  void set_img(const std::string& name, cv::Mat&& img);
private:
  mutable std::mutex mutex_;
  std::unordered_map<std::string, std::shared_ptr<cv::Mat>> img_map_;
};

} // namespace detector