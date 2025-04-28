#include "detector/detector.hpp"

namespace detector
{
  detector::detector()
  {
    // 打开默认摄像头
    cap_.open(0, cv::CAP_V4L2);
    if (!cap_.isOpened())
    {
      throw std::runtime_error("Camera open failed");
    }

    // 设置摄像头参数
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap_.set(cv::CAP_PROP_FPS, 30);
  }

  detector::~detector()
  {
    stop_capture();
    if (cap_.isOpened())
    {
      cap_.release();
    }
  }

  void detector::start_capture()
  {
    if (capturing_)
      return;
    capturing_ = true;
    capture_thread_ = std::thread(&detector::main_loop, this);
  }

  void detector::stop_capture()
  {
    capturing_ = false;
    if (capture_thread_.joinable())
    {
      capture_thread_.join();
    }
  }

  bool detector::is_capturing() const
  {
    return capturing_;
  }

  void detector::main_loop()
  {
    while (capturing_)
    {
      cv::Mat frame;
      if (cap_.read(frame))
      {
        // 在锁外完成耗时操作
        auto new_frame = std::make_shared<cv::Mat>(std::move(frame));

        cv::imshow("Debug", *new_frame);
        cv::waitKey(1);
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }
} // namespace detector