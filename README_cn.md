# 基于 RDKx5 的智慧实验室系统

`lab_inspection` 是一个 ROS2 包，实现了基于 RDKx5 的智慧实验室巡检系统。它集成了 Nav2 导航、视觉检测、AI推理、华为云 IoTDA 上报、L610 4G 模组控制，以及可选的大模型二次判断。

## 主要功能

- ROS2 包含 `navigation_node` 和 `vision_node`
- 基于 Nav2 的自主巡航：AMCL、map_server、planner、controller、behavior server
- 视觉巡检模型：`yolov8n.bin`
- 自定义 `/trigger_inspection` 服务触发巡检
- 通过 L610 4G 模组和华为 IoTDA 进行 MQTT 上报
- 可选 Qwen 大模型辅助判断模糊目标
- 自动发布 `base_link` → `laser_link` 静态 TF，支持兼容的激光雷达布局

## 仓库结构

- `CMakeLists.txt` — ROS2 包编译与安装配置
- `package.xml` — 包清单与依赖声明
- `launch/lab_inspection.launch.py` — 启动机器人底层、Nav2、视觉与导航节点
- `config/` — Nav2 参数、YOLOv8/视觉参数、导航配置
- `maps/` — 实验室地图文件
- `models/` — 视觉模型文件，支持 `.bin` 硬件部署
- `scripts/vision_py.py` — Python 视觉节点，支持双模型推理与云上报
- `src/navigation_node.cpp` — C++ 导航节点，负责巡检工位与服务交互
- `src/vision_node.cpp` — C++ 视觉节点，负责底层硬件与推理部署
- `srv/TriggerInspection.srv` — 自定义巡检服务定义

## 构建步骤

1. Source ROS2 环境：

```bash
source /opt/ros/<ros2-distro>/setup.bash
```

2. 安装系统依赖：

- `ament_cmake`
- `rosidl_default_generators`
- `ament_cmake_python`
- `rclcpp`, `rclpy`, `rclcpp_action`, `rclcpp_components`
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `nav2_msgs`, `std_msgs`, `std_srvs`
- `tf2_ros`, `cv_bridge`
- `nlohmann_json`
- `OpenCV`
- `libcurl`

3. 安装 Python 依赖：

```bash
python3 -m pip install numpy opencv-python  pyserial openai
```

4. 使用 colcon 编译：

```bash
colcon build --packages-select lab_inspection
```

5. Source 本地安装：

```bash
source install/setup.bash
```

## 启动方式

```bash
ros2 launch lab_inspection lab_inspection.launch.py
```

可选参数：

```bash
ros2 launch lab_inspection lab_inspection.launch.py initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

该启动文件将执行：

- 启动机器人底层 bringup，加载相机、激光雷达和 IMU
- 发布 `base_link` -> `laser_link` 静态 TF
- 启动 Nav2 栈：map_server、AMCL、planner、controller、behavior server、lifecycle manager
- 启动 `vision_py.py` 视觉巡检节点
- 启动 `navigation_node` 巡检导航协调节点

## 运行服务

- `/trigger_inspection` (`lab_inspection/srv/TriggerInspection`) — 发起视觉巡检
- `/start_inspection` — 启动巡检流程
- `/skip_station` — 跳过当前工位
- `/connect_huawei_cloud` — 连接华为云
- `/disconnect_huawei_cloud` — 断开华为云

`TriggerInspection` 服务定义：

```srv
int32 station_id
---
bool success
string message
```

## 配置文件说明

- `config/nav2_params.yaml` — Nav2 参数与生命周期配置
- `config/navigate_no_replan.xml` — Nav2 行为树配置
- `config/yolov8_config.yaml` — YOLOv8 与视觉参数配置
- `maps/lab_map.yaml` — Nav2 地图文件
- `models/yolov8n.bin` — `.bin` 视觉模型

## 硬件依赖说明

- 适配基于 RDKx5 的机器人平台，兼容现有机器人 bringup 包
- 当 `hb_dnn` 库存在时，支持 BPU 加速；x86 平台可在无 BPU 情况下降级运行
- 需要机器人摄像头驱动推送的图像话题，如 `/image`
- 使用 L610 4G 模组和 UART 控制实现华为云上报
- 需要华为设备凭证和 Qwen API Key 用于云上报与可选大模型判断

