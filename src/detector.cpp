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
    // 使用 Inference 类初始化模型
    arrow_inference_ = std::make_unique<Inference>(
        "/home/temp/project/good/install/detector/share/detector/models/arrow.onnx",
        cv::Size(640, 640), // 模型输入尺寸
        "",                 // 类别文件路径，如果没有可以传空字符串
        false               // 是否使用 CUDA
    );

    cylida_inference_ = std::make_unique<Inference>(
        "/home/temp/project/good/install/detector/share/detector/models/cylida.onnx",
        cv::Size(640, 640),
        "",
        false);
    net_arrow_ = cv::dnn::readNetFromONNX("/home/temp/project/good/install/detector/share/detector/models/arrow.onnx");
    if (net_arrow_.empty())
    {
      throw std::runtime_error("Failed to load ONNX model");
    }
    net_cylida_ = cv::dnn::readNetFromONNX("/home/temp/project/good/install/detector/share/detector/models/cylida.onnx");
    if (net_cylida_.empty())
    {
      throw std::runtime_error("Failed to load ONNX model");
    }
    current_frame_ = cv::Mat(480, 640, CV_8UC3);
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
  const cv::Mat detector::return_frame() const
  {
    std::lock_guard<std::mutex> lock(frame_mutex_); 
    if (current_frame_.empty())
    {
      return cv::Mat();
    }
    return current_frame_;
  }

  void detector::capture_thread_func()
  {
    cv::Mat frame;
    while (capturing_)
    {
      if (cap_.read(frame))
      {
        if (cap_.read(frame))
        {
          cv::flip(frame, frame, -1);

          // 调用推理（箭头+圆柱体检测）
          //auto arrow_dets = arrow_inference_->runInference(frame);
           auto cylida_dets = cylida_inference_->runInference(frame);

          // 合并结果
          std::vector<Detection> all_dets;
          //all_dets.insert(all_dets.end(), arrow_dets.begin(), arrow_dets.end());
           all_dets.insert(all_dets.end(), cylida_dets.begin(), cylida_dets.end());

          // 绘制检测框（直接使用Detection中的颜色和类别）
          cv::Mat display = frame.clone();
          for (const auto &det : all_dets)
          {
            std::cout<<det.className.c_str()<<" "<< det.confidence<<std::endl;
            cv::rectangle(display, det.box, det.color, 2);
            std::string label = cv::format("%s %.2f", det.className.c_str(), det.confidence);
            cv::putText(display, label, det.box.tl() + cv::Point(0, -5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, det.color, 2);
          }

          // 更新当前帧
          {
            std::lock_guard<std::mutex> lock(frame_mutex_);
            current_frame_ = display.clone();
          }
        }
        else
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      }
    }
  }
} // namespace detector