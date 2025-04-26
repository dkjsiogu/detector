cmake_minimum_required(VERSION 3.10)
project(detector)

# 基础配置
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Werror)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

#######################
## Find dependencies ##
#######################
find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV 4.5 REQUIRED COMPONENTS core imgproc)
# 自动查找所有在package.xml中声明的依赖
ament_auto_find_build_dependencies()

###########
## Build ##
###########

# 1. 编译纯算法静态库（仍需手动配置）
add_library(detector_lib STATIC
  src/detector.cpp
)
target_include_directories(detector_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(detector_lib PUBLIC
  ${OpenCV_LIBS}
  opencv_core
  ament_index_cpp::ament_index_cpp
  opencv_videoio # 关键：解决VideoCapture符号问题
  opencv_imgproc
  opencv_highgui
)

# 2. 自动构建ROS 2节点组件
ament_auto_add_library(detector_component SHARED
  src/detector_node.cpp
)

# 补充手动配置（ament_auto未覆盖的部分）
target_include_directories(detector_component PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(detector_component
  detector_lib
  image_transport::image_transport
  ${OpenCV_LIBS}
)

# 3. 组件注册
rclcpp_components_register_nodes(detector_component
  "detector_node::detector_node"
)

#############
## Install ##
#############
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY 
  res/
  DESTINATION share/${PROJECT_NAME}/
)
# 自动安装规则（会安装所有通过ament_auto_add_*添加的目标）
ament_auto_package(
  INSTALL_TO_SHARE
  include
  # 可以添加其他需要安装的目录，如config、launch等
)

# 手动安装非自动构建的目标
install(TARGETS detector_lib
  DESTINATION lib/${PROJECT_NAME}
)
