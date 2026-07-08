cmake_minimum_required(VERSION 3.8)
project(lab_inspection)

# 开启 C++17 标准支持，以匹配现代 ROS2 C++ 和 nlohmann/json 的特性
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# =========================================================================
# 1. 查找系统依赖包与标准库
# =========================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nlohmann_json REQUIRED)

find_package(nav_msgs REQUIRED)        # Odometry 消息类型
find_package(tf2_ros REQUIRED)         # TransformBroadcaster

# ===== 以下为 package.xml 中声明但缺失的依赖 =====
find_package(sensor_msgs REQUIRED)      # 传感器消息类型
find_package(cv_bridge REQUIRED)        # OpenCV与ROS图像转换
find_package(std_msgs REQUIRED)         # 标准ROS消息类型
# ================================================

# 真实图像处理与大模型Base64压缩必选：查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 真实端到端网络通信必选：查找系统级 libcurl 库
find_package(CURL REQUIRED)

# 地瓜 RDK 平台 BPU 原生底层硬件加速算子运行时依赖库 (板卡端自带)
# 仅在 RDK 板卡上可用，x86 平台上跳过
find_library(DNN_LIB name dnn HINTS /usr/lib /usr/local/lib)
set(HAS_BPU TRUE)
if(DNN_LIB STREQUAL "DNN_LIB-NOTFOUND")
  message(STATUS "hb_dnn not found (not on RDK platform), building without BPU support (vision_node skipped)")
  set(DNN_LIB "")
  set(HAS_BPU FALSE)
else()
  message(STATUS "Found DNN library: ${DNN_LIB}")
endif()

# =========================================================================
# 2. 注册和自动生成自定义服务接口 (TriggerInspection.srv)
# =========================================================================
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/TriggerInspection.srv"
)

# =========================================================================
# 3. 头文件包含路径配置
# =========================================================================
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

# =========================================================================
# 4. 创建可执行二进制目标并绑定全链路底层库
# =========================================================================

# 获取 ROS2 自动生成的接口目标，用于节点间通信 C++ 类型支持
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# A. 创建你新编写的本地 BPU 真部署、大模型真实联网、L610串口控制综合主节点
add_executable(vision_node src/vision_node.cpp)
# 挂载 ament 框架级 ROS2 核心依赖
ament_target_dependencies(vision_node
  rclcpp
  rclcpp_components
  geometry_msgs
  std_srvs
  nlohmann_json
  sensor_msgs      # 新增：vision_node 可能用到图像消息
  cv_bridge        # 新增：OpenCV与ROS图像转换
  std_msgs         # 新增：标准消息类型
)
# 强制链接：物理 BPU 推理动态库 + 真实网络请求 CURL 库 + 图像处理 OpenCV 核心库 + 内部自定义服务通讯接口
target_link_libraries(vision_node
  ${OpenCV_LIBS}
  ${DNN_LIB}
  ${CURL_LIBRARIES}
  ${cpp_typesupport_target}
)

# B. 创建你队友编写的工位导航节点（自动化多点定位巡航核心）
add_executable(navigation_node src/navigation_node.cpp)
ament_target_dependencies(navigation_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
  nav_msgs
  tf2_ros
  std_srvs
  nlohmann_json
  std_msgs         # 新增：标准消息类型
)
target_link_libraries(navigation_node ${cpp_typesupport_target})

# =========================================================================
# 5. 目标与资产安装目录配置 (保证 install 后能通过 ros2 run/launch 正常拉起)
# =========================================================================

# 安装所有的编译可执行二进制文件到 lib 目录下
install(TARGETS
  navigation_node
  vision_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件、启动脚本、地图、本地 .bin 硬件推理模型至工作区 share 目录
install(DIRECTORY config launch maps models
  DESTINATION share/${PROJECT_NAME}
)

# 安装 Python 视觉节点(双模型并集)
install(PROGRAMS scripts/vision_py.py
  DESTINATION lib/${PROJECT_NAME}
)

# =========================================================================
# 6. Ament 功能包打包输出
# =========================================================================
ament_export_dependencies(rosidl_default_runtime)
ament_package()