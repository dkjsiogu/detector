#include "detector/detector_node.hpp"

namespace detector_node
{
    detector_node::detector_node(const rclcpp::NodeOptions &options)
        : Node("detector_node", options),
          it_(std::make_shared<image_transport::ImageTransport>(
              std::shared_ptr<detector_node::Node>(this, [](auto *) {})))
    {
        // 启动 OpenCV 捕获线程
        det_.start_capture();

        // 创建 ROS2 图像发布者
        pub_ = this->create_publisher<std_msgs::msg::String>("detection_result", 10);
        pub_img_ = it_->advertise("camera/image_raw", 1);
        // 串口初始化
        serial_com::SerialPort::Config serial_config;
        serial_config.port = this->declare_parameter("serial_port", "/dev/ttyUSB0");
        serial_config.baudrate = this->declare_parameter("baudrate", 115200);
        serial_ = std::make_unique<serial_com::SerialPort>(*this, serial_config);

        // 设置接收回调
        serial_->set_callback([this](const std::vector<uint8_t> &data)
                              {
                                  RCLCPP_INFO(this->get_logger(), "Received %zu bytes", data.size());
                                  // 处理接收到的数据...
                              });
        // 定时器（33ms ≈ 30FPS）
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33),
            [this]()
            { this->publishFrame(); });

        RCLCPP_INFO(this->get_logger(), "Publishing camera feed");
    }

    detector_node::~detector_node()
    {
        det_.stop_capture();
        // 停止串口通信
        if (serial_)
        {
            serial_->stop(); // 假设你在 SerialPort 类中实现了 close()
            serial_.reset();
        }


        RCLCPP_INFO(this->get_logger(), "detector_node shutdown complete.");
    }

    void detector_node::publishFrame()
    {

        auto frame = det_.return_frame();
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame captured");
            return;
        }

        // 使用 cv_bridge 转换图像
        auto msg = cv_bridge::CvImage(
                       std_msgs::msg::Header(), // 消息头
                       "bgr8",                  // OpenCV 默认 BGR 格式
                       frame                    // 输入图像
                       )
                       .toImageMsg();

        // 设置时间戳和坐标系
        msg->header.stamp = this->now();
        msg->header.frame_id = "camera";

        // 发布消息 (使用 image_transport 发布者)
        pub_img_.publish(msg);

        auto packet = serial_->create_packet()
                          .add(0xAA) // 起始标志
                          .add(2)
                          .add(0xBB) // 结束标志
                          .build();

        if (!serial_->send(packet))
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to send detection data");
        }
    }
} // namespace detector_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector_node::detector_node)