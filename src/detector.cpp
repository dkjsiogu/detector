#include <detector/detector.hpp>

namespace detector
{
  detector::detector::detector()
  {
    std::string image_path = ament_index_cpp::get_package_share_directory("detector") + "/Figure_1.png";
    set_img("raw", std::move(cv::imread(image_path)));

  }
  std::shared_ptr<const cv::Mat> detector::get_img(const std::string &name) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = img_map_.find(name);
    return (it != img_map_.end()) ? it->second : nullptr;
  }

  void detector::set_img(const std::string &name, cv::Mat &&img)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    img_map_[name] = std::make_shared<cv::Mat>(std::move(img));
  }
}
