#include "detector/detector_node.hpp"

namespace detector_node
{
  detector_node::detector_node(const rclcpp::NodeOptions &options)
      : Node("detector_node", options)
  {
    /*打开默认摄像头
    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
      throw std::runtime_error("Camera open failed");
    }

    // 设置摄像头参数
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 60);
    */ 
    // 创建 ROS2 图像发布者
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 定时器（16ms ≈ 60FPS）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(16),
        [this]()
        { this->publishFrame(); });

    RCLCPP_INFO(this->get_logger(), "Publishing camera feed (using cv_bridge)");
  }

  void detector_node::publishFrame()
  {
    auto frame=this->det_.get_img("raw");

    if (!frame ||frame->empty())
    {
      RCLCPP_WARN(this->get_logger(), "Empty frame captured");
      return;
    }

    // 使用 cv_bridge 转换图像
    auto msg = cv_bridge::CvImage(
                   std_msgs::msg::Header(), // 消息头
                   "bgr8",                  // OpenCV 默认 BGR 格式
                   *frame                    // 输入图像
                   )
                   .toImageMsg();

    // 设置时间戳和坐标系
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";

    // 发布消息
    pub_->publish(*msg);
  }
} // namespace detector_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector_node::detector_node)