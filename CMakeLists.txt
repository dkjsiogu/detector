cmake_minimum_required(VERSION 3.10)
project(detector)

# 使用C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用严格编译检查
add_definitions(-Wall -Werror)

# 为IDE提供编译命令
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.5 REQUIRED
  COMPONENTS core imgproc  # 明确指定需要的OpenCV模块
)
find_package(image_transport REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# 添加可执行文件（替代原来的detector_node）
ament_auto_add_executable(camera_publisher
  src/detector_node.cpp
)

# 包含目录和链接库
target_include_directories(camera_publisher PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(camera_publisher
  ${OpenCV_LIBS}
  image_transport::image_transport
)

# 组件注册（如果适用）
# rclcpp_components_register_node(camera_publisher
#   PLUGIN your_package::CameraPublisher
# )

#############
## Install ##
#############

ament_auto_package(
  INSTALL_TO_SHARE
  # 如有需要可以添加其他安装内容
  # config
  # launch
  # model
)