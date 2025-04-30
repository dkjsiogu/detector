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
    set_img("g",cv::Mat());
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
          *img_map_["result"]=frame.clone();
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
        std::cout<<direction;
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
    for (const auto &contour : contours) {
        double area = cv::contourArea(contour);
        
        // 绘制所有轮廓（红色表示被淘汰的）
        cv::drawContours(*img_map_["result"], contours, &contour - &contours[0], 
                        (area < 500) ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0), 2);
        
        if (area < 1500) continue;

        cv::RotatedRect rect = cv::minAreaRect(contour);
        float aspect_ratio = rect.size.width / rect.size.height;
        
        // 绘制最小外接矩形（黄色）
        cv::Point2f vertices[4];
        rect.points(vertices);
        for(int i = 0; i < 4; i++) {
            cv::line(*img_map_["result"], vertices[i], vertices[(i+1)%4], 
                    cv::Scalar(0, 255, 255), 2);
        }

        if (aspect_ratio < 0.5 || aspect_ratio > 2.0) continue;

        // 凸性检测（箭头通常有凸缺陷）
        std::vector<int> hull;
        cv::convexHull(contour, hull);
        
        // 绘制凸包点（紫色）
        for(size_t j = 0; j < hull.size(); j++) {
            cv::circle(*img_map_["result"], contour[hull[j]], 5, 
                      cv::Scalar(255, 0, 255), -1);
        }

        if (hull.size() > 5 && hull.size()<7) {
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

    // 确保result图像已初始化
    if (img_map_["result"]->empty()) {
        *img_map_["result"] = cv::Mat::zeros(480, 640, CV_8UC3);
    }

    // 方法1：最小外接矩形方向
    cv::RotatedRect rect = cv::minAreaRect(contour);
    float angle = rect.angle;
    if (rect.size.width < rect.size.height) angle += 90;

    // 绘制矩形中心（蓝色）
    cv::circle(*img_map_["result"], rect.center, 8, cv::Scalar(255, 0, 0), -1);

    // 方法2：主成分分析(PCA)
    cv::Mat data_pts = cv::Mat(contour.size(), 2, CV_64F);
    for (size_t i = 0; i < contour.size(); i++) {
        data_pts.at<double>(i, 0) = contour[i].x;
        data_pts.at<double>(i, 1) = contour[i].y;
    }
    cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);
    cv::Point2d eigen_vec = pca.eigenvectors.at<cv::Vec2d>(0);
    cv::Point2d mean = pca.mean.at<cv::Vec2d>(0);
    double pca_angle = atan2(eigen_vec.y, eigen_vec.x) * 180 / CV_PI;

    // 绘制PCA主方向（红色）
    cv::line(*img_map_["result"], mean, 
             mean + eigen_vec * 100, 
             cv::Scalar(0, 0, 255), 2);

    // 综合判定
    double final_angle = (angle + pca_angle) / 2;

    // 绘制角度文本
    std::string angle_text = "Angle: " + std::to_string(int(final_angle));
    cv::putText(*img_map_["result"], angle_text, cv::Point(10, 30), 
               cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 255, 255), 2);

    // 绘制方向指示箭头
    cv::Point center(rect.center.x, rect.center.y);
    cv::Point tip;
    ArrowDirection dir;
    
    if (final_angle > -45 && final_angle <= 45) {
        dir = RIGHT;
        tip = center + cv::Point(50, 0);
    } else if (final_angle > 45 && final_angle <= 135) {
        dir = DOWN;
        tip = center + cv::Point(0, 50);
    } else if (final_angle > 135 || final_angle <= -135) {
        dir = LEFT;
        tip = center + cv::Point(-50, 0);
    } else {
        dir = UP;
        tip = center + cv::Point(0, -50);
    }
    
    // 绘制方向箭头（白色）
    cv::arrowedLine(*img_map_["result"], center, tip, 
                   cv::Scalar(255, 255, 255), 3, cv::LINE_AA, 0, 0.1);

    return dir;
  }
} // namespace detector