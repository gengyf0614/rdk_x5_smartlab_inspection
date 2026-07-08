# Smart Laboratory Inspection System Based on RDKx5

`lab_inspection` is a ROS 2 package that implements a smart laboratory inspection system based on RDKx5. It integrates Nav2 navigation, visual detection, AI inference, Huawei IoTDA reporting, L610 4G module control, and an optional large-language-model-based secondary judgment.

## Main Features

- ROS 2 package containing `navigation_node` and `vision_node`
- Autonomous inspection navigation based on Nav2: AMCL, map_server, planner, controller, and behavior server
- Visual inspection model: `yolov8n.bin`
- Custom `/trigger_inspection` service for initiating inspection tasks
- MQTT reporting through the L610 4G module and Huawei IoTDA
- Optional Qwen large-model assistance for ambiguous target judgment
- Automatic publishing of the static TF from `base_link` to `laser_link`, supporting compatible lidar layouts

## Repository Structure

- `CMakeLists.txt` — ROS 2 package build and installation configuration
- `package.xml` — Package manifest and dependency declarations
- `launch/lab_inspection.launch.py` — Starts the robot base, Nav2, vision, and navigation nodes
- `config/` — Nav2 parameters, YOLOv8/vision parameters, and navigation configuration
- `maps/` — Laboratory map files
- `models/` — Vision model files, supporting `.bin` deployment
- `scripts/vision_py.py` — Python vision node supporting dual-model inference and cloud reporting
- `src/navigation_node.cpp` — C++ navigation node responsible for inspection station coordination and service interaction
- `src/vision_node.cpp` — C++ vision node responsible for low-level hardware access and inference deployment
- `srv/TriggerInspection.srv` — Custom inspection service definition

## Build Steps

1. Source the ROS 2 environment:

```bash
source /opt/ros/<ros2-distro>/setup.bash
```

2. Install system dependencies:

- `ament_cmake`
- `rosidl_default_generators`
- `ament_cmake_python`
- `rclcpp`, `rclpy`, `rclcpp_action`, `rclcpp_components`
- `geometry_msgs`, `sensor_msgs`, `nav_msgs`, `nav2_msgs`, `std_msgs`, `std_srvs`
- `tf2_ros`, `cv_bridge`
- `nlohmann_json`
- `OpenCV`
- `libcurl`

3. Install Python dependencies:

```bash
python3 -m pip install numpy opencv-python pyserial openai
```

4. Build with colcon:

```bash
colcon build --packages-select lab_inspection
```

5. Source the local install:

```bash
source install/setup.bash
```

## Launch Instructions

```bash
ros2 launch lab_inspection lab_inspection.launch.py
```

Optional arguments:

```bash
ros2 launch lab_inspection lab_inspection.launch.py initial_x:=0.0 initial_y:=0.0 initial_yaw:=0.0
```

This launch file will:

- Start the robot bringup stack, loading the camera, lidar, and IMU
- Publish the static TF from `base_link` to `laser_link`
- Start the Nav2 stack: map_server, AMCL, planner, controller, behavior server, and lifecycle manager
- Start the `vision_py.py` inspection node
- Start the `navigation_node` inspection coordination node

## Service Usage

- `/trigger_inspection` (`lab_inspection/srv/TriggerInspection`) — Initiates a visual inspection
- `/start_inspection` — Starts the inspection workflow
- `/skip_station` — Skips the current station
- `/connect_huawei_cloud` — Connects to Huawei Cloud
- `/disconnect_huawei_cloud` — Disconnects from Huawei Cloud

The `TriggerInspection` service definition is:

```srv
int32 station_id
---
bool success
string message
```

## Configuration Files

- `config/nav2_params.yaml` — Nav2 parameters and lifecycle configuration
- `config/navigate_no_replan.xml` — Nav2 behavior tree configuration
- `config/yolov8_config.yaml` — YOLOv8 and vision parameter configuration
- `maps/lab_map.yaml` — Nav2 map file
- `models/yolov8n.bin` — Vision model file in `.bin` format

## Hardware Requirements

- Compatible with RDKx5-based robot platforms and existing robot bringup packages
- Supports BPU acceleration when the `hb_dnn` library is available; x86 platforms can run in a degraded mode without BPU
- Requires image topics published by the robot camera driver, such as `/image`
- Uses the L610 4G module and UART control for Huawei cloud reporting
- Requires Huawei device credentials and a Qwen API key for cloud reporting and optional large-model inference
