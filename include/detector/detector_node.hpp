#pragma once
#include <detector/detector.hpp>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
namespace detector_node {

class detector_node : public rclcpp::Node {
public:
  explicit detector_node(const rclcpp::NodeOptions & options);
  ~detector_node();
private:
  void publishFrame();  // 定时回调函数声明
  detector::detector det_;
  // 成员变量
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::atomic<uint64_t> last_published_sequence_{0};
};

} // namespace detector_node