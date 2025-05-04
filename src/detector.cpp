#include "detector/detector.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

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

    init_img_map();

    // 加载预设图像
    std::unordered_map<std::string, std::string> img_paths;
    img_paths["toward"] = ament_index_cpp::get_package_share_directory("detector") + "/toward.jpeg";
    img_paths["right"] = ament_index_cpp::get_package_share_directory("detector") + "/right.jpeg";

    set_img("toward", cv::imread(img_paths["toward"]));
    set_img("right", cv::imread(img_paths["right"]));
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

  void detector::init_img_map()
  {
    // 预分配内存
    set_img("raw", cv::Mat(480, 640, CV_8UC3));
    set_img("result", cv::Mat());
    set_img("g", cv::Mat());
  }

  std::shared_ptr<const cv::Mat> detector::detector::return_frame()
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    auto it = img_map_.find("raw");
    return (it != img_map_.end()) ? it->second : nullptr;
  }

  std::shared_ptr<const cv::Mat> detector::get_img(const std::string &name) const
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    auto it = img_map_.find(name);
    return (it != img_map_.end()) ? it->second : nullptr;
  }

  void detector::set_img(const std::string &name, cv::Mat &&img)
  {
    std::lock_guard<std::mutex> lock(img_mutex_);
    img_map_[name] = std::make_shared<cv::Mat>(std::move(img));
  }

  void detector::start_capture()
  {
    if (capturing_)
      return;

    capturing_ = true;
    capture_thread_ = std::thread(&detector::capture_thread_func, this);
    process_thread_ = std::thread(&detector::process_thread_func, this);
    display_thread_ = std::thread(&detector::display_thread_func, this);
  }

  void detector::stop_capture()
  {
    capturing_ = false;

    if (capture_thread_.joinable())
      capture_thread_.join();
    if (process_thread_.joinable())
      process_thread_.join();
    if (display_thread_.joinable())
      display_thread_.join();
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
        {
          std::unique_lock<std::mutex> lock(img_mutex_);
          frame_processed_ = false;
          *img_map_["raw"] = frame.clone();
          *img_map_["result"] = frame.clone();
        }
        frame_ready_.notify_one();
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
  }

  void detector::process_thread_func()
  {
    cv::Mat hsv, mask;
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    while (capturing_)
    {
      {
        std::unique_lock<std::mutex> lock(img_mutex_);
        frame_ready_.wait(lock, [this]()
                          { return !capturing_ || !frame_processed_; });

        if (!capturing_)
          break;

        // 处理流水线
        cv::cvtColor(*img_map_["raw"], hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, lower_green_, upper_green_, mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        *img_map_["g"] = mask.clone();

        auto contour = findArrowContour(mask);
        auto direction = determineDirection(contour);
        std::cout << direction;
        frame_processed_ = true;
      }
    }
  }

  void detector::display_thread_func()
  {
    while (capturing_)
    {
      {
        std::lock_guard<std::mutex> lock(img_mutex_);
        if (!img_map_["result"]->empty())
        {
          cv::imshow("Debug", *img_map_["result"]);
        }
      }

      if (cv::waitKey(30) == 27)
      {
        capturing_ = false;
      }
    }
  }

  std::vector<cv::Point> detector::findArrowContour(const cv::Mat &binary_mask)
  {

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 筛选条件：面积、长宽比、凸性等
    for (const auto &contour : contours)
    {
      double area = cv::contourArea(contour);

      // 绘制所有轮廓（红色表示被淘汰的）
      cv::drawContours(*img_map_["result"], contours, &contour - &contours[0],
                       (area < 500) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);

      if (area<500)
        continue;

      cv::RotatedRect rect = cv::minAreaRect(contour);
      float aspect_ratio = rect.size.width / rect.size.height;

      // 绘制最小外接矩形（黄色）
      cv::Point2f vertices[4];
      rect.points(vertices);
      for (int i = 0; i < 4; i++)
      {
        cv::line(*img_map_["result"], vertices[i], vertices[(i + 1) % 4],
                 cv::Scalar(0, 255, 255), 2);
      }

      if (aspect_ratio < 0.5 || aspect_ratio > 2.0)
        continue;

      // 凸性检测（箭头通常有凸缺陷）
      std::vector<int> hull;
      cv::convexHull(contour, hull);

      // 绘制凸包点（紫色）
      for (size_t j = 0; j < hull.size(); j++)
      {
        cv::circle(*img_map_["result"], contour[hull[j]], 5,
                   cv::Scalar(255, 0, 255), -1);
      }

      if (hull.size() > 15 && hull.size() < 50)
      {
        // 标记最终选中的轮廓（青色）
        cv::drawContours(*img_map_["result"], std::vector<std::vector<cv::Point>>{contour},
                         0, cv::Scalar(255, 255, 0), 3);
        return contour;
      }
    }

    return {};
  }

  ArrowDirection detector::determineDirection(const std::vector<cv::Point> &contour)
  {
    if (contour.empty()) return UNKNOWN;

    // 1. 计算最小外接矩形和凸包
    //cv::RotatedRect rect = cv::minAreaRect(contour);
    std::vector<int> hull;
    cv::convexHull(contour, hull);

    // 2. 寻找箭头的三角形部分（凸缺陷最深的区域）
    std::vector<cv::Vec4i> defects;
    cv::convexityDefects(contour, hull, defects);
    
    if (defects.empty()) return UNKNOWN;

    // 找到最深的凸缺陷（箭头的"凹口"）
    auto max_defect = *std::max_element(defects.begin(), defects.end(),
        [](const cv::Vec4i& a, const cv::Vec4i& b) {
            return a[3] < b[3]; // 比较缺陷深度
        });

    // 3. 确定三角形顶点（凹口两侧的点）
    cv::Point triangle_top = contour[max_defect[2]]; // 凹口最深点
    cv::Point left_edge = contour[max_defect[0]];    // 凹口左边缘
    cv::Point right_edge = contour[max_defect[1]];   // 凹口右边缘

    // 4. 计算箭头的指向方向
    cv::Point2f arrow_dir;
    if (cv::norm(triangle_top - left_edge) > cv::norm(triangle_top - right_edge)) {
        // 箭头指向left_edge方向
        arrow_dir = cv::Point2f(left_edge - triangle_top);
    } else {
        // 箭头指向right_edge方向
        arrow_dir = cv::Point2f(right_edge - triangle_top);
    }
    
    // 规范化方向向量
    arrow_dir /= cv::norm(arrow_dir);
    arrow_dir=-arrow_dir;
    // 5. 计算角度并确定具体方向
    double angle = atan2(arrow_dir.y, arrow_dir.x) * 180 / CV_PI;
    angle = fmod(angle + 360, 360); // 规范化到0-360度

    // 6. 可视化调试（可选）
    if (!img_map_["result"]->empty()) {
        // 绘制凸缺陷
        cv::line(*img_map_["result"], left_edge, triangle_top, cv::Scalar(0,255,255), 2);
        cv::line(*img_map_["result"], right_edge, triangle_top, cv::Scalar(0,255,255), 2);
        
        // 绘制指向方向
        cv::arrowedLine(*img_map_["result"], triangle_top, 
                       triangle_top + cv::Point(arrow_dir * 50), 
                       cv::Scalar(255,0,0), 3);
    }

    // 7. 方向判定
    if (angle >= 315 || angle < 45) return RIGHT;
    if (angle >= 45 && angle < 135) return DOWN;
    if (angle >= 135 && angle < 225) return LEFT;
    return UP;
  }
} // namespace detector