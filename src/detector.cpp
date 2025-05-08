#include "detector/detector.hpp"

namespace detector
{

  detector::detector()
  {
    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
      throw std::runtime_error("Camera open failed");
    }

    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);
    net_arrow_ = cv::dnn::readNetFromONNX("models/arrow.onnx");
    if (net_arrow_.empty()) {
        throw std::runtime_error("Failed to load ONNX model");
    }
    net_cylida_ = cv::dnn::readNetFromONNX("models/cylida.onnx");
    if (net_cylida_.empty()) {
        throw std::runtime_error("Failed to load ONNX model");
    }
  }

  detector::~detector()
  {
    stop_capture();
    if (cap_.isOpened())
    {
      cap_.release();
    }
    cv::destroyAllWindows();
  }

  void detector::start_capture()
  {
    if (capturing_)
      return;

    capturing_ = true;
    capture_thread_ = std::thread(&detector::capture_thread_func, this);
  }

  void detector::stop_capture()
  {
    capturing_ = false;

    if (capture_thread_.joinable())
      capture_thread_.join();
  }

  bool detector::is_capturing() const
  {
    return capturing_;
  }

  void detector::capture_thread_func()
  {
    cv::Mat frame;
    while (capturing_)
    {
      if (cap_.read(frame))
      { 
        cv::flip(frame, frame, -1);
         cv::Mat blob = cv::dnn::blobFromImage(frame, 1.0, cv::Size(64, 64));
            net_arrow_.setInput(blob);
            cv::Mat outputs = net_arrow_.forward();
            
            // 解析结果
            cv::Point classIdPoint;
            double confidence;
            cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);
            int class_id = classIdPoint.x;
            
            // 显示结果
            std::string label = cv::format("%s: %.2f", class_names_[class_id].c_str(), confidence);
            cv::putText(frame, label, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
            
            cv::imshow("Debug", frame);
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
  std::string detector::detector::get_result(ArrowDirection Dir)
  {
    switch (Dir)
    {
    case UNKNOWN:
      return "UNKNOWN";
    case UP:
      return "UP";
    case RIGHT:
      return "RIGHT";
    case LEFT:
      return "LEFT";

    default:
      break;
    }
  }
} // namespace detector