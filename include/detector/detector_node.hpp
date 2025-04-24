#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <detector/detector.hpp>
namespace detector_node {

class detector_node : public rclcpp::Node {
public:
  explicit detector_node(const rclcpp::NodeOptions & options);
private:
  void publishFrame();  // 定时回调函数声明

  // 成员变量
  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace detector_node