#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {
    // 打开默认摄像头
    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open camera!");
      throw std::runtime_error("Camera open failed");
    }

    // 设置摄像头参数
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 60);

    // 创建ROS2图像发布者
    pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera/image_raw", 10);

    // 定时器（33ms ≈ 30FPS）
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(33),
      std::bind(&CameraPublisher::publishFrame, this)
    );

    RCLCPP_INFO(this->get_logger(), "Publishing camera feed (no cv_bridge)");
  }

private:
  void publishFrame() {
    cv::Mat frame;
    cap_ >> frame;

    if (frame.empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty frame captured");
      return;
    }

    // 手动构建ROS2 Image消息
    auto msg = std::make_shared<sensor_msgs::msg::Image>();
    
    // 设置消息头
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera";
    
    // 图像参数
    msg->height = frame.rows;
    msg->width = frame.cols;
    msg->encoding = "bgr8";  // OpenCV默认BGR格式
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
    
    // 复制图像数据
    msg->data.assign(frame.datastart, frame.dataend);

    // 发布消息
    pub_->publish(*msg);
  }

  cv::VideoCapture cap_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}