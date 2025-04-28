#include "detector/detector_node.hpp"

namespace detector_node {
detector_node::detector_node(const rclcpp::NodeOptions &options)
    : Node("detector_node", options) {
    // 启动 OpenCV 捕获线程
    det_.start_capture();

    // 创建 ROS2 图像发布者
    pub_ = this->create_publisher<std_msgs::msg::String>("detection_result", 10);

    // 定时器（33ms ≈ 30FPS）
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(33),
        [this]() { this->publishFrame(); });

    RCLCPP_INFO(this->get_logger(), "Publishing camera feed");
}

detector_node::~detector_node() {
    det_.stop_capture();
}

void detector_node::publishFrame() {
    
}
} // namespace detector_node

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(detector_node::detector_node)