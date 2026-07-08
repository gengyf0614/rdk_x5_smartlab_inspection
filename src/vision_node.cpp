#include "lab_inspection/vision_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <ctime>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <sensor_msgs/image_encodings.hpp>

using json = nlohmann::json;

namespace lab_inspection
{

static const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// ==================== AT 命令字符串转义 ====================
// AT 命令中 quoted string 内的 " \ , 需要转义，否则解析器会截断参数
static std::string escape_at_string(const std::string & raw)
{
  std::string escaped;
  escaped.reserve(raw.size() + 8);
  for (char c : raw) {
    switch (c) {
      case '"':  escaped += "\\\""; break;  // " → \"
      case '\\': escaped += "\\\\"; break;  // \ → \\
      case ',':  escaped += "\\,";  break;  // , → \,
      default:   escaped += c;      break;
    }
  }
  return escaped;
}

// AT 命令中已被双引号包裹的字符串只需要转义 " 和 \（逗号被引号保护，无需转义）
// 用于 JSON payload 等已在引号内的内容，避免 \, 破坏 JSON 结构
static std::string escape_quoted_at_string(const std::string & raw)
{
  std::string escaped;
  escaped.reserve(raw.size() + 8);
  for (char c : raw) {
    switch (c) {
      case '"':  escaped += "\\\""; break;  // " → \"
      case '\\': escaped += "\\\\"; break;  // \ → \\
      default:   escaped += c;      break;
    }
  }
  return escaped;
}

// ==================== 标签映射（根据实际训练标签） ====================
struct ClassInfo {
    int id;
    std::string name;
    std::string chinese_name;
    bool is_abnormal;      // 是否为异常状态（需要告警）
    bool need_llm_judge;   // 是否需要大模型二次研判
};

static const std::vector<ClassInfo> kClassInfoList = {
    {0, "chair_untidy", "过道椅子未归位", true, false},
    {1, "osc_on",       "示波器开机",     true, false},
    {2, "osc_off",      "示波器关机",     false, false},
    {3, "siggen_on",    "信号发生器开机", true, false},
    {4, "siggen_off",   "信号发生器关机", false, false},
    {5, "psu_on",       "电源开机",       true, false},
    {6, "psu_off",      "电源关机",       false, false},
    {7, "dmm_on",       "万用表开机",     true, false},
    {8, "dmm_off",      "万用表关机",     false, false},
    {9, "mess",         "桌面杂乱区域",   true, true}
};

static const ClassInfo* get_class_info(int id) {
    for (const auto& info : kClassInfoList) {
        if (info.id == id) return &info;
    }
    return nullptr;
}

static bool is_abnormal_state(int id) {
    const auto* info = get_class_info(id);
    return info ? info->is_abnormal : false;
}

static bool need_llm_judge(int id) {
    const auto* info = get_class_info(id);
    return info ? info->need_llm_judge : false;
}

static std::string get_chinese_name(int id) {
    const auto* info = get_class_info(id);
    return info ? info->chinese_name : "未知目标";
}

// ==================== 构造函数 ====================
VisionNode::VisionNode(const rclcpp::NodeOptions & options)
: Node("vision_node", options)
{
  RCLCPP_INFO(this->get_logger(), "======> [物理硬核部署] 正在注入真实硬件参数...");

  // 1. 声明并强绑定参数服务器里的物理字段
  this->declare_parameter<std::string>("model_path", "src/lab_inspection/models/yolov8n.bin");
  this->declare_parameter<double>("conf_threshold", 0.35);
  this->declare_parameter<double>("io_threshold", 0.45);
  this->declare_parameter<std::string>("camera_topic", "/image");
  this->declare_parameter<int>("camera_queue_size", 5);
  this->declare_parameter<std::string>("huawei_mqtt_server", "121.36.42.100");
  this->declare_parameter<int>("huawei_mqtt_port", 8883);
  this->declare_parameter<std::string>("qwen_api_key", "");
  this->declare_parameter<std::string>("uart_device", "/dev/ttyUSB0");
  this->declare_parameter<int>("uart_baudrate", 115200);
  this->declare_parameter<std::vector<std::string>>("forgotten_items_dict", std::vector<std::string>{});
  this->declare_parameter<std::string>("device_id", "6110e20e0ad1ed0286438504_lab_inspection");
  this->declare_parameter<std::string>("device_secret", "zr6r03181996");

  this->get_parameter("model_path", model_path_);
  this->get_parameter("conf_threshold", conf_threshold_);
  this->get_parameter("io_threshold", io_threshold_);
  this->get_parameter("camera_topic", camera_topic_);
  this->get_parameter("camera_queue_size", camera_queue_size_);
  this->get_parameter("huawei_mqtt_server", huawei_mqtt_server_);
  this->get_parameter("huawei_mqtt_port", huawei_mqtt_port_);
  this->get_parameter("qwen_api_key", qwen_api_key_);
  this->get_parameter("uart_device", uart_device_);
  this->get_parameter("uart_baudrate", uart_baudrate_);
  this->get_parameter("forgotten_items_dict", forgotten_items_dict_);
  this->get_parameter("device_id", device_id_);
  this->get_parameter("device_secret", device_secret_);

  // 警告：API Key不应硬编码
  if (qwen_api_key_.empty() || qwen_api_key_ == "sk-33a2d028f38847b6a625f3f3efdcac84") {
    RCLCPP_WARN(this->get_logger(), "请通过环境变量或安全方式设置 qwen_api_key，不要硬编码在yaml中！");
  }

  // 2. 动态加载 yaml 里的 station_contacts 分组手机号字典
  load_contacts_from_yaml();
  
  // 3. 驱动底层的物理链路
  init_bpu_model();
  init_l610_uart();
  init_camera_subscription();

  inspection_service_ = this->create_service<lab_inspection::srv::TriggerInspection>(
    "/trigger_inspection",
    std::bind(&VisionNode::handle_inspection, this, std::placeholders::_1, std::placeholders::_2));

  // 华为云连接管理服务（供 navigation_node 在巡逻开始/结束时调用）
  connect_cloud_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/connect_huawei_cloud",
    std::bind(&VisionNode::connect_cloud_callback, this, std::placeholders::_1, std::placeholders::_2));
  disconnect_cloud_service_ = this->create_service<std_srvs::srv::Trigger>(
    "/disconnect_huawei_cloud",
    std::bind(&VisionNode::disconnect_cloud_callback, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "全链路真外设通信无缝打通，系统进入待命状态。");
}

VisionNode::~VisionNode()
{
  if (packed_model_) hbDNNRelease(packed_model_);
  if (input_properties_) delete[] input_properties_;
  if (output_properties_) delete[] output_properties_;
  if (uart_fd_ >= 0) close(uart_fd_);
  RCLCPP_INFO(this->get_logger(), "全解耦物理句柄安全关闭。");
}

// ==================== 动态解析YAML联系人映射 ====================
void VisionNode::load_contacts_from_yaml()
{
  RCLCPP_INFO(this->get_logger(), "正在动态提取 yaml 里的外部联系人映射网络...");
  
  std::string prefix = "station_contacts";
  auto result = this->list_parameters({prefix}, 2);

  for (const auto & param_name : result.names) {
    size_t last_dot = param_name.find_last_of('.');
    if (last_dot != std::string::npos) {
      std::string station_sub = param_name.substr(last_dot + 1);
      size_t underscore = station_sub.find_last_of('_');
      if (underscore != std::string::npos) {
        try {
          int id = std::stoi(station_sub.substr(underscore + 1));
          std::vector<std::string> phones;
          this->get_parameter(param_name, phones);
          station_contacts_map_[id] = phones;
          RCLCPP_INFO(this->get_logger(), " 工位ID [%d] 绑定手机数: %zu", id, phones.size());
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), " 解析工位ID失败: %s", e.what());
        }
      }
    }
  }
}

// ==================== 摄像头订阅（通过 originbot hobot_usb_cam 发布的 ROS 话题获取图像） ====================
void VisionNode::init_camera_subscription()
{
  RCLCPP_INFO(this->get_logger(), " 订阅 originbot 摄像头话题: %s (队列=%d)", 
              camera_topic_.c_str(), camera_queue_size_);
  
  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, camera_queue_size_,
    std::bind(&VisionNode::image_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), " 摄像头话题订阅成功，等待 originbot hobot_usb_cam 推送图像...");
}

void VisionNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(image_mutex_);
  try {
    // 先按原始编码获取图像，再统一转为 BGR（OpenCV 推理需要 BGR 格式）
    cv::Mat raw = cv_bridge::toCvCopy(msg, msg->encoding)->image;
    if (msg->encoding == sensor_msgs::image_encodings::RGB8) {
      cv::cvtColor(raw, latest_image_, cv::COLOR_RGB2BGR);
    } else if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
      latest_image_ = raw;
    } else {
      // 其他编码尝试转为 BGR
      cv::cvtColor(raw, latest_image_, cv::COLOR_RGB2BGR);
    }
    if (!latest_image_.empty()) {
      has_image_ = true;
    }
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), " cv_bridge 图像转换失败: %s", e.what());
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), " OpenCV 颜色转换失败: %s", e.what());
  }
}

// ==================== BPU模型初始化（正确版本） ====================
void VisionNode::init_bpu_model()
{
  RCLCPP_INFO(this->get_logger(), " 正在加载BPU模型: %s", model_path_.c_str());
  
  // 1. 加载打包模型文件
  const char* model_files[] = { model_path_.c_str() };
  int ret = hbDNNInitializeFromFiles(&packed_model_, model_files, 1);
  if (ret != 0) {
    RCLCPP_FATAL(this->get_logger(), 
      " 地瓜 BPU 推理核心加载模型失败！错误码: %d, 路径: %s", 
      ret, model_path_.c_str());
    throw std::runtime_error("BPU load failed");
  }
  
  // 2. 获取模型名称列表
  const char **model_names;
  int model_count;
  ret = hbDNNGetModelNameList(&model_names, &model_count, packed_model_);
  if (ret != 0 || model_count == 0) {
    RCLCPP_FATAL(this->get_logger(), " 获取模型名称失败！");
    throw std::runtime_error("BPU get model name failed");
  }
  
  RCLCPP_INFO(this->get_logger(), " BPU模型加载成功，模型名称: %s", model_names[0]);
  
  // 3. 获取模型句柄
  ret = hbDNNGetModelHandle(&model_handle_, packed_model_, model_names[0]);
  if (ret != 0 || !model_handle_) {
    RCLCPP_FATAL(this->get_logger(), " 获取模型句柄失败！");
    throw std::runtime_error("BPU get model handle failed");
  }
  
  // 4. 获取模型输入输出数量
  ret = hbDNNGetInputCount(&input_count_, model_handle_);
  if (ret != 0) {
    RCLCPP_FATAL(this->get_logger(), " 获取模型输入数量失败！");
    throw std::runtime_error("BPU get input count failed");
  }
  ret = hbDNNGetOutputCount(&output_count_, model_handle_);
  if (ret != 0) {
    RCLCPP_FATAL(this->get_logger(), " 获取模型输出数量失败！");
    throw std::runtime_error("BPU get output count failed");
  }
  
  RCLCPP_INFO(this->get_logger(), " BPU模型信息: 输入数量=%d, 输出数量=%d", input_count_, output_count_);
  
  // 5. 分配并获取输入属性
  input_properties_ = new hbDNNTensorProperties[input_count_];
  for (int i = 0; i < input_count_; i++) {
    ret = hbDNNGetInputTensorProperties(&input_properties_[i], model_handle_, i);
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), " 获取模型输入属性[%d]失败！", i);
      throw std::runtime_error("BPU get input properties failed");
    }
    
    // 打印输入张量信息
    auto& shape = input_properties_[i].validShape;
    RCLCPP_INFO(this->get_logger(), 
      "   输入[%d]: 维度=[%d,%d,%d,%d], 类型=%d",
      i,
      shape.dimensionSize[0],
      shape.dimensionSize[1],
      shape.dimensionSize[2],
      shape.dimensionSize[3],
      input_properties_[i].tensorType);
  }
  
  // 6. 分配并获取输出属性
  output_properties_ = new hbDNNTensorProperties[output_count_];
  for (int i = 0; i < output_count_; i++) {
    ret = hbDNNGetOutputTensorProperties(&output_properties_[i], model_handle_, i);
    if (ret != 0) {
      RCLCPP_FATAL(this->get_logger(), " 获取模型输出属性[%d]失败！", i);
      throw std::runtime_error("BPU get output properties failed");
    }
    
    // 打印输出张量信息
    auto& shape = output_properties_[i].validShape;
    std::string dims_str;
    for (int d = 0; d < shape.numDimensions; d++) {
      dims_str += std::to_string(shape.dimensionSize[d]);
      if (d < shape.numDimensions - 1) dims_str += "x";
    }
    RCLCPP_INFO(this->get_logger(), 
      "  输出[%d]: 维度=%s, 类型=%d",
      i, dims_str.c_str(), output_properties_[i].tensorType);
  }
}

// ==================== L610串口初始化 ====================
void VisionNode::init_l610_uart()
{
  uart_fd_ = open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (uart_fd_ < 0) {
    RCLCPP_FATAL(this->get_logger(), "!!! 无法打开物理串口总线 %s !!!", uart_device_.c_str());
    throw std::runtime_error("UART device open failed");
  }
  
  struct termios options;
  tcgetattr(uart_fd_, &options);
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);
  options.c_cflag |= (CLOCAL | CREAD);
  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 10;
  tcsetattr(uart_fd_, TCSANOW, &options);
  tcflush(uart_fd_, TCIOFLUSH);

  // 0. 关闭 AT 命令回显（防止后续响应解析被回显误导）
  send_l610_at_command("ATE0", "OK", 1000);
  
  // 1. 检查模块响应
  if(!send_l610_at_command("AT", "OK", 1000)) {
    RCLCPP_ERROR(this->get_logger(), "广和通 L610 硬件无应答！请检查串口接线");
    throw std::runtime_error("L610 not responding");
  }
  RCLCPP_INFO(this->get_logger(), "广和通 L610模块响应正常");

  // 2. 检查SIM卡状态
  if(!send_l610_at_command("AT+CPIN?", "READY", 3000)) {
    RCLCPP_ERROR(this->get_logger(), "SIM卡未就绪，请检查SIM卡");
    throw std::runtime_error("SIM card not ready");
  }

  // 3. 检查信号质量
  std::string csq_reply;
  if(send_l610_at_command("AT+CSQ", "CSQ:", 3000, &csq_reply)) {
    RCLCPP_INFO(this->get_logger(), "信号质量: %s", csq_reply.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "无法获取信号质量");
  }

  // 4. 检查网络注册状态
  std::string cereg_reply;
  if(send_l610_at_command("AT+CEREG?", "+CEREG:", 5000, &cereg_reply)) {
    if(cereg_reply.find("0,1") != std::string::npos || 
       cereg_reply.find("0,5") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "网络已注册 (EPS附着成功)");
    } else {
      RCLCPP_WARN(this->get_logger(), "网络未注册，状态: %s", cereg_reply.c_str());
    }
  }

  // 5. 查询 PDP 状态（由 MIPCALL 统一管理，不做 CGACT，避免冲突）
  std::string mipcall_reply;
  if(send_l610_at_command("AT+MIPCALL?", "+MIPCALL:", 3000, &mipcall_reply)) {
    RCLCPP_INFO(this->get_logger(), "PDP初始状态: %s", mipcall_reply.c_str());
  }

  mqtt_connected_ = false;
}

bool VisionNode::send_l610_at_command(const std::string & cmd, const std::string & expected_reply, int timeout_ms)
{
  return send_l610_at_command(cmd, expected_reply, timeout_ms, nullptr);
}

// 辅助函数：支持获取回复内容的重载
bool VisionNode::send_l610_at_command(const std::string & cmd, const std::string & expected_reply, int timeout_ms, std::string* out_reply)
{
  if (uart_fd_ < 0) return false;
  tcflush(uart_fd_, TCIOFLUSH);
  std::string full_cmd = cmd + "\r\n";
  write(uart_fd_, full_cmd.c_str(), full_cmd.length());

  std::string reply = "";
  char buf[128];
  int elapsed = 0;
  const int interval_ms = 10;

  while (elapsed < timeout_ms) {
    int len = read(uart_fd_, buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      reply += buf;
      if (reply.find(expected_reply) != std::string::npos) {
        if (out_reply) *out_reply = reply;
        return true;
      }
    } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
      RCLCPP_ERROR(this->get_logger(), "串口读取出错, errno: %d", errno);
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_ms));
    elapsed += interval_ms;
  }
  RCLCPP_WARN(this->get_logger(), "AT命令超时: %s. 实际接收到: %s", cmd.c_str(), reply.c_str());
  return false;
}

// ==================== 连接华为云（L610 AT+HMCON 命令） ====================
// AT+HMCON=<session>,<keepalive>,"<server>","<port>","<deviceId>","<deviceSecret>",<flag>
// 前置条件: 必须先通过 AT+MIPCALL=1 拿到 IP 地址
bool VisionNode::connect_huawei_cloud()
{
  if (mqtt_connected_) return true;

  // 1. 确保 PDP 已激活、已分配到 IP
  //    使用 MIPCALL 统一管理（不要混用 CGACT，会冲突报 ERROR）
  std::string ipcall_reply;
  if (send_l610_at_command("AT+MIPCALL?", "+MIPCALL:", 3000, &ipcall_reply)) {
    RCLCPP_INFO(this->get_logger(), "PDP状态: %s", ipcall_reply.c_str());
    // +MIPCALL: 0   → 未激活, 需要发 AT+MIPCALL=1
    // +MIPCALL: 1,<ip> → 已激活, 有IP
    if (ipcall_reply.find("+MIPCALL: 1,") == std::string::npos &&
        ipcall_reply.find("+MIPCALL:1,") == std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "PDP未激活, 发送 AT+MIPCALL=1 请求IP...");
      // 直接用串口发命令, 然后等 URC +MIPCALL: <ip>
      tcflush(uart_fd_, TCIOFLUSH);
      write(uart_fd_, "AT+MIPCALL=1\r\n", 14);
      // 等待 +MIPCALL: <ip> URC（模块会先回 OK, 再异步上报 IP）
      std::string urc_buf;
      char buf[128];
      int elapsed = 0;
      bool got_ip = false;
      while (elapsed < 15000) {
        int len = read(uart_fd_, buf, sizeof(buf) - 1);
        if (len > 0) {
          buf[len] = '\0';
          urc_buf += buf;
          // 检查是否收到了 IP 地址
          if (urc_buf.find("+MIPCALL:") != std::string::npos &&
              urc_buf.find("+MIPCALL: 0") == std::string::npos) {
            got_ip = true;
            break;
          }
          if (urc_buf.find("ERROR") != std::string::npos) {
            RCLCPP_WARN(this->get_logger(), "MIPCALL=1 失败: %s", urc_buf.c_str());
            break;
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        elapsed += 50;
      }
      if (got_ip) {
        RCLCPP_INFO(this->get_logger(), "PDP激活成功, URC: %s", urc_buf.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "未收到IP URC, 继续尝试连接...");
      }
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } else {
      RCLCPP_INFO(this->get_logger(), "已有IP, 无需重新激活");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "MIPCALL查询失败, 尝试直接连接...");
  }

  // 2. 构建 AT+HMCON 连接命令（字符串需转义，防止设备ID/密钥中的特殊字符截断参数）
  std::string hmcon_cmd = "AT+HMCON=0,60,\"" + escape_at_string(huawei_mqtt_server_) + "\"," +
                          "\"" + escape_at_string(std::to_string(huawei_mqtt_port_)) + "\"," +
                          "\"" + escape_at_string(device_id_) + "\"," +
                          "\"" + escape_at_string(device_secret_) + "\"," +
                          "0";

  RCLCPP_INFO(this->get_logger(), "正在连接华为云: server=%s:%d, deviceId=%s",
              huawei_mqtt_server_.c_str(), huawei_mqtt_port_, device_id_.c_str());

  if (!send_l610_at_command(hmcon_cmd, "+HMCON OK", 20000)) {
    RCLCPP_ERROR(this->get_logger(),
      "HMCON 连接失败! 请确认: 1)SIM卡有流量 2)服务器地址正确 3)设备已注册 4)密钥正确");
    return false;
  }

  mqtt_connected_ = true;
  mqtt_msg_id_ = 1;
  RCLCPP_INFO(this->get_logger(), "华为云 MQTT 连接成功 (+HMCON OK)");

  return true;
}

// ==================== 断开华为云（正确流程：先断MQTT，再释放IP） ====================
// AT+HMDISC=0  →  +HMDIS OK      (断开MQTT连接，释放session资源)
// AT+MIPCALL=0  →  OK → +MIPCALL: 0  (释放PDP IP地址)
// 注意: L610 对 HMDISC 的响应是 +HMDIS OK (没有字母C)
//       断开后如需再次发布消息，必须重新执行设备认证和连接
void VisionNode::disconnect_huawei_cloud()
{
  if (!mqtt_connected_) {
    RCLCPP_INFO(this->get_logger(), "华为云未连接，跳过断开流程");
    return;
  }

  if (uart_fd_ < 0) {
    RCLCPP_WARN(this->get_logger(), "串口未打开，强制标记为未连接");
    mqtt_connected_ = false;
    return;
  }

  // 步骤1: 断开 MQTT 连接并释放 session 资源
  RCLCPP_INFO(this->get_logger(), "正在断开华为云 MQTT 连接 (AT+HMDISC=0)...");
  if (send_l610_at_command("AT+HMDISC=0", "+HMDIS OK", 10000)) {
    mqtt_connected_ = false;
    RCLCPP_INFO(this->get_logger(), "华为云 MQTT 已断开 (+HMDIS OK)");
  } else {
    RCLCPP_WARN(this->get_logger(), "HMDISC 无响应，强制标记为未连接");
    mqtt_connected_ = false;
  }

  // 步骤2: 释放 PDP IP 地址
  RCLCPP_INFO(this->get_logger(), "正在释放 PDP IP 地址 (AT+MIPCALL=0)...");
  std::string mipcall_reply;
  if (send_l610_at_command("AT+MIPCALL=0", "+MIPCALL:", 10000, &mipcall_reply)) {
    if (mipcall_reply.find("+MIPCALL: 0") != std::string::npos ||
        mipcall_reply.find("+MIPCALL:0") != std::string::npos) {
      RCLCPP_INFO(this->get_logger(), "PDP IP 释放成功 (+MIPCALL: 0)");
    } else {
      RCLCPP_INFO(this->get_logger(), "MIPCALL 响应: %s", mipcall_reply.c_str());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "MIPCALL=0 无响应，IP可能未被释放");
  }

  RCLCPP_INFO(this->get_logger(), "华为云断开流程完成（MQTT已断开 + IP已释放）");
}

// ==================== 华为云连接管理服务回调 ====================
void VisionNode::connect_cloud_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "收到云端连接请求");
  bool ok = connect_huawei_cloud();
  response->success = ok;
  response->message = ok ? "华为云连接成功" : "华为云连接失败";
}

void VisionNode::disconnect_cloud_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "收到云端断开请求");
  disconnect_huawei_cloud();
  response->success = true;
  response->message = "华为云已断开";
}

// ==================== IoU 计算 ====================
float VisionNode::compute_iou(const cv::Rect& box1, const cv::Rect& box2)
{
  int x1 = std::max(box1.x, box2.x);
  int y1 = std::max(box1.y, box2.y);
  int x2 = std::min(box1.x + box1.width, box2.x + box2.width);
  int y2 = std::min(box1.y + box1.height, box2.y + box2.height);
  int inter_w = std::max(0, x2 - x1);
  int inter_h = std::max(0, y2 - y1);
  float inter_area = static_cast<float>(inter_w * inter_h);
  float box1_area = static_cast<float>(box1.width * box1.height);
  float box2_area = static_cast<float>(box2.width * box2.height);
  float union_area = box1_area + box2_area - inter_area;
  if (union_area <= 0.0f) return 0.0f;
  return inter_area / union_area;
}

// ==================== NMS非极大值抑制 ====================
std::vector<DetectionResult> VisionNode::apply_nms(
    std::vector<DetectionResult>& detections, 
    float nms_threshold)
{
  if (detections.empty()) {
    return {};
  }
  
  // 按置信度排序
  std::sort(detections.begin(), detections.end(),
    [](const DetectionResult& a, const DetectionResult& b) {
      return a.score > b.score;
    });
  
  std::vector<bool> keep(detections.size(), true);
  
  for (size_t i = 0; i < detections.size(); i++) {
    if (!keep[i]) continue;
    
    for (size_t j = i + 1; j < detections.size(); j++) {
      if (!keep[j]) continue;
      
      float iou = compute_iou(detections[i].bbox, detections[j].bbox);
      if (iou > nms_threshold) {
        keep[j] = false;
      }
    }
  }
  
  std::vector<DetectionResult> result;
  for (size_t i = 0; i < detections.size(); i++) {
    if (keep[i]) {
      result.push_back(detections[i]);
    }
  }
  
  return result;
}

// ==================== 调试函数：打印输出张量信息 ====================
void VisionNode::debug_print_output_info(void* output_data, const hbDNNTensorProperties& properties, size_t output_size)
{
  RCLCPP_INFO(this->get_logger(), "========== BPU输出调试信息 ==========");
  
  // 打印维度
  std::string dims_str;
  size_t total_elements = 1;
  for (int d = 0; d < properties.validShape.numDimensions; d++) {
    int dim = properties.validShape.dimensionSize[d];
    dims_str += std::to_string(dim);
    if (d < properties.validShape.numDimensions - 1) dims_str += "x";
    total_elements *= dim;
  }
  RCLCPP_INFO(this->get_logger(), "输出维度: %s", dims_str.c_str());
  RCLCPP_INFO(this->get_logger(), "总元素数: %zu", total_elements);
  RCLCPP_INFO(this->get_logger(), "内存大小: %zu bytes", output_size);
  
  if (properties.tensorType != HB_DNN_TENSOR_TYPE_F32 || output_size < sizeof(float)) {
    RCLCPP_INFO(this->get_logger(), "=====================================");
    return;
  }
  
  float* fdata = static_cast<float*>(output_data);
  int total_floats = static_cast<int>(output_size / sizeof(float));
  
  // 打印前20个原始值
  {
    std::string values_str;
    int print_count = std::min(20, total_floats);
    for (int i = 0; i < print_count; i++) {
      values_str += std::to_string(fdata[i]) + (i < print_count-1 ? ", " : "");
    }
    RCLCPP_INFO(this->get_logger(), "前%d个原始值: [%s]", print_count, values_str.c_str());
  }
  
  // 推断通道数和预测数，按通道采样打印
  int num_dims = properties.validShape.numDimensions;
  int num_channels = 0, num_predictions = 0;
  
  if (num_dims == 4) {
    int d1 = properties.validShape.dimensionSize[1];
    int d2 = properties.validShape.dimensionSize[2];
    int d3 = properties.validShape.dimensionSize[3];
    // [1, C, P, 1] 格式: C 较小, P 较大
    if (d1 <= 200 && d2 > 200)      { num_channels = d1; num_predictions = d2; }
    else if (d2 <= 200 && d1 > 200) { num_channels = d2; num_predictions = d1; }
    else if (d3 <= 200 && d2 > 200) { num_channels = d3; num_predictions = d2; }
  } else if (num_dims == 3) {
    int d1 = properties.validShape.dimensionSize[1];
    int d2 = properties.validShape.dimensionSize[2];
    if (d1 <= 200 && d2 > 200)      { num_channels = d1; num_predictions = d2; }
    else if (d2 <= 200 && d1 > 200) { num_channels = d2; num_predictions = d1; }
  }
  
  if (num_channels >= 4 && num_predictions > 0) {
    int total_floats = static_cast<int>(output_size / sizeof(float));
    
    // 同时打印两种布局的前3个预测，方便对比
    int max_pred = std::min(3, num_predictions);
    
    // ---- channel_first 解读: fdata[c * num_predictions + p] ----
    RCLCPP_INFO(this->get_logger(), "--- channel_first 解读 (fdata[c*%d+p]) ---", num_predictions);
    for (int p = 0; p < max_pred; p++) {
      std::string pred_str;
      for (int c = 0; c < num_channels; c++) {
        int idx = c * num_predictions + p;
        if (idx < total_floats) {
          pred_str += (c >= 4 ? "cls" + std::to_string(c-4) + "=" : "") 
                   + std::to_string(fdata[idx]);
          if (c < num_channels - 1) pred_str += ", ";
        }
      }
      RCLCPP_INFO(this->get_logger(), "  cf_pred[%d]: [%s]", p, pred_str.c_str());
    }
    
    // ---- predictions_first 解读: fdata[p * num_channels + c] ----
    RCLCPP_INFO(this->get_logger(), "--- predictions_first 解读 (fdata[p*%d+c]) ---", num_channels);
    for (int p = 0; p < max_pred; p++) {
      std::string pred_str;
      for (int c = 0; c < num_channels; c++) {
        int idx = p * num_channels + c;
        if (idx < total_floats) {
          pred_str += (c >= 4 ? "cls" + std::to_string(c-4) + "=" : "") 
                   + std::to_string(fdata[idx]);
          if (c < num_channels - 1) pred_str += ", ";
        }
      }
      RCLCPP_INFO(this->get_logger(), "  pf_pred[%d]: [%s]", p, pred_str.c_str());
    }
    
    // 分别统计两种布局的 class score 非零率
    int num_classes = num_channels - 4;
    if (num_classes > 0) {
      auto stats = [&](bool cf, const char* label) {
        int above = 0;
        float gmax = 0.0f;
        float gmin = 1e9f;
        for (int p = 0; p < num_predictions; p++) {
          float max_cls = 0.0f;
          float min_cls = 1e9f;
          for (int c = 4; c < num_channels; c++) {
            float v = cf ? fdata[c * num_predictions + p] : fdata[p * num_channels + c];
            if (v > max_cls) max_cls = v;
            if (v < min_cls) min_cls = v;
          }
          if (max_cls > gmax) gmax = max_cls;
          if (min_cls < gmin) gmin = min_cls;
          if (max_cls >= 0.35f) above++;
        }
        RCLCPP_INFO(this->get_logger(),
          "  [%s] cls_range=[%.2f, %.2f], ≥0.35=%d/%d",
          label, gmin, gmax, above, num_predictions);
      };
      stats(true, "channel_first ");
      stats(false, "preds_first  ");
    }
  }
  
  RCLCPP_INFO(this->get_logger(), "=====================================");
}

// ==================== BPU输出解析（数据驱动布局检测 + sigmoid） ====================
std::vector<DetectionResult> VisionNode::parse_yolo_output(
    void* output_data,
    const hbDNNTensorProperties& properties,
    int original_width, int original_height,
    int model_width, int model_height)
{
  std::vector<DetectionResult> results;
  
  if (properties.tensorType != HB_DNN_TENSOR_TYPE_F32) {
    RCLCPP_ERROR(this->get_logger(), "不支持的输出数据类型: %d", properties.tensorType);
    return results;
  }
  
  float* output = static_cast<float*>(output_data);
  int num_dims = properties.validShape.numDimensions;
  
  // ---- 从张量维度中提取通道数和预测数 ----
  int num_channels = 0;
  int num_predictions = 0;
  
  if (num_dims == 4) {
    int d1 = properties.validShape.dimensionSize[1];
    int d2 = properties.validShape.dimensionSize[2];
    int d3 = properties.validShape.dimensionSize[3];
    // 找到 ≤200 的维度作为通道数, >200 的维度作为预测数
    int dims[] = {d1, d2, d3};
    for (int i = 0; i < 3; i++) {
      if (dims[i] > 200 && dims[i] > num_predictions) num_predictions = dims[i];
      if (dims[i] >= 4 && dims[i] <= 200 && dims[i] > num_channels) num_channels = dims[i];
    }
  } else if (num_dims == 3) {
    int d1 = properties.validShape.dimensionSize[1];
    int d2 = properties.validShape.dimensionSize[2];
    if (d1 <= 200 && d2 > 200)      { num_channels = d1; num_predictions = d2; }
    else if (d2 <= 200 && d1 > 200) { num_channels = d2; num_predictions = d1; }
  }
  
  if (num_channels < 5 || num_predictions < 1 || num_predictions > 200000) {
    RCLCPP_ERROR(this->get_logger(), "YOLO参数异常: ch=%d, pred=%d", num_channels, num_predictions);
    return results;
  }
  
  const int box_attrs = 4;
  int num_classes = num_channels - box_attrs;  // 假设 YOLOv8 格式 (无独立 obj)
  
  // ====== 数据驱动布局检测：采样 class score 位置，判断真实内存布局 ======
  // channel_first: cls[c][p] = output[c * num_predictions + p]
  // preds_first:   cls[c][p] = output[p * num_channels + (box_attrs + c)]
  // 正确布局下 class scores 应有非零值；错误布局可能全是 0
  int sample_count = std::min(20, num_predictions);
  float cf_sum = 0.0f, pf_sum = 0.0f;
  int cf_nonzero = 0, pf_nonzero = 0;
  
  for (int s = 0; s < sample_count; s++) {
    int p = s * (num_predictions / sample_count);  // 均匀采样
    for (int c = 0; c < num_classes; c++) {
      int ch = box_attrs + c;
      // channel_first 读法
      float v_cf = output[ch * num_predictions + p];
      cf_sum += std::abs(v_cf);
      if (std::abs(v_cf) > 0.001f) cf_nonzero++;
      // predictions_first 读法
      float v_pf = output[p * num_channels + ch];
      pf_sum += std::abs(v_pf);
      if (std::abs(v_pf) > 0.001f) pf_nonzero++;
    }
  }
  
  bool channel_first;
  if (pf_nonzero > cf_nonzero * 2) {
    channel_first = false;  // predictions_first 有更多非零值 → 选它
  } else if (cf_nonzero > pf_nonzero * 2) {
    channel_first = true;
  } else {
    // 无法从数据判断，回退到形状启发式
    int ch_dim = -1, pred_dim = -1;
    if (num_dims == 4) {
      int d1 = properties.validShape.dimensionSize[1];
      int d2 = properties.validShape.dimensionSize[2];
      if (d1 == num_channels) ch_dim = 1;
      if (d2 == num_channels) ch_dim = 2;
      if (d1 == num_predictions) pred_dim = 1;
      if (d2 == num_predictions) pred_dim = 2;
      channel_first = (ch_dim < pred_dim);
    } else {
      channel_first = true;
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "布局检测: cf_nonzero=%d pf_nonzero=%d → channel_first=%d",
    cf_nonzero, pf_nonzero, channel_first);
  
  int total_attrs = num_channels;
  
  // ====== 判断 class scores 是否需要 sigmoid ======
  // 采样几个 class score，如果值 > 2.0 或 < -2.0，说明是 raw logits，需要 sigmoid
  bool need_sigmoid = false;
  {
    float max_abs_cls = 0.0f;
    for (int s = 0; s < std::min(100, num_predictions); s++) {
      int p = s;
      for (int c = 0; c < num_classes; c++) {
        int ch = box_attrs + c;
        float v = channel_first ? output[ch * num_predictions + p]
                                : output[p * num_channels + ch];
        if (std::abs(v) > max_abs_cls) max_abs_cls = std::abs(v);
      }
    }
    need_sigmoid = (max_abs_cls > 5.0f);  // 类概率应在 [0,1]，>5 说明是 logits
    RCLCPP_INFO(this->get_logger(), 
      "class_score 前100预测 max_abs=%.2f → need_sigmoid=%d", max_abs_cls, need_sigmoid);
  }
  
  // 内联 sigmoid
  auto sigmoid = [](float x) { return 1.0f / (1.0f + std::exp(-x)); };
  
  // ====== 逐预测解析 ======
  int detected_raw = 0;
  for (int p = 0; p < num_predictions; p++) {
    // 找最大 class score
    float best_score = -1e9f;
    int best_class = 0;
    for (int c = 0; c < num_classes; c++) {
      int ch = box_attrs + c;
      float raw = channel_first ? output[ch * num_predictions + p]
                                : output[p * num_channels + ch];
      float score = need_sigmoid ? sigmoid(raw) : raw;
      if (score > best_score) {
        best_score = score;
        best_class = c;
      }
    }
    
    if (best_score >= conf_threshold_) {
      detected_raw++;
      
      // 读取 bbox 坐标（假定已经是模型像素空间的值）
      float cx, cy, w, h;
      if (channel_first) {
        cx = output[0 * num_predictions + p];
        cy = output[1 * num_predictions + p];
        w  = output[2 * num_predictions + p];
        h  = output[3 * num_predictions + p];
      } else {
        cx = output[p * num_channels + 0];
        cy = output[p * num_channels + 1];
        w  = output[p * num_channels + 2];
        h  = output[p * num_channels + 3];
      }
      
      // 如果需要 sigmoid，bbox 坐标也可能需要（YOLOv8 cx,cy 经 sigmoid 后解码）
      // 这里简单处理：如果值远超模型尺寸，尝试做 sigmoid + 缩放
      float cx_f = cx, cy_f = cy, w_f = w, h_f = h;
      if (need_sigmoid) {
        // cx, cy 可能也需要 sigmoid（YOLOv8 标准做法）
        // 但如果值已经在合理像素范围内，就保持原样
        if (cx_f > model_width || cx_f < -model_width) {
          cx_f = sigmoid(cx) * model_width;
        }
        if (cy_f > model_height || cy_f < -model_height) {
          cy_f = sigmoid(cy) * model_height;
        }
        if (w_f > model_width * 2 || w_f < 0) {
          w_f = sigmoid(w) * model_width;
        }
        if (h_f > model_height * 2 || h_f < 0) {
          h_f = sigmoid(h) * model_height;
        }
      }
      
      // 映射: 模型空间 → 原始图像空间
      int x1 = static_cast<int>((cx_f - w_f * 0.5f) * original_width / model_width);
      int y1 = static_cast<int>((cy_f - h_f * 0.5f) * original_height / model_height);
      int x2 = static_cast<int>((cx_f + w_f * 0.5f) * original_width / model_width);
      int y2 = static_cast<int>((cy_f + h_f * 0.5f) * original_height / model_height);
      
      x1 = std::max(0, std::min(x1, original_width - 1));
      y1 = std::max(0, std::min(y1, original_height - 1));
      x2 = std::max(0, std::min(x2, original_width - 1));
      y2 = std::max(0, std::min(y2, original_height - 1));
      
      if (x2 > x1 && y2 > y1) {
        DetectionResult det;
        det.id = best_class;
        det.score = best_score;
        det.bbox = cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2));
        results.push_back(det);
      }
    }
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "YOLO检出: %d/%d (conf≥%.2f), NMS前=%zu, need_sigmoid=%d",
    detected_raw, num_predictions, conf_threshold_, results.size(), need_sigmoid);
  
  // NMS
  if (!results.empty()) {
    size_t before = results.size();
    results = apply_nms(results, 0.45f);
    RCLCPP_INFO(this->get_logger(), "NMS后: %zu (过滤%zu)", results.size(), before - results.size());
  }
  
  return results;
}

// ==================== BPU推理执行（正确版本） ====================
std::vector<DetectionResult> VisionNode::execute_bpu_inference(const cv::Mat & frame)
{
  std::lock_guard<std::mutex> lock(bpu_mutex_);
  std::vector<DetectionResult> results;
  
  if (!model_handle_ || frame.empty()) {
    return results;
  }
  
  // 获取模型期望的输入尺寸
  auto& input_shape = input_properties_[0].validShape;
  int expected_height = input_shape.dimensionSize[2];
  int expected_width = input_shape.dimensionSize[3];
  int expected_channels = input_shape.dimensionSize[1];
  
  RCLCPP_DEBUG(this->get_logger(), "模型期望输入: %dx%dx%d", expected_width, expected_height, expected_channels);
  
  // 预处理：缩放
  cv::Mat resized_img;
  cv::resize(frame, resized_img, cv::Size(expected_width, expected_height));
  
  // 根据模型要求转换色彩空间
  cv::Mat rgb_img;
  if (input_properties_[0].tensorType == HB_DNN_IMG_TYPE_RGB) {
    cv::cvtColor(resized_img, rgb_img, cv::COLOR_BGR2RGB);
  } else {
    rgb_img = resized_img;
  }
  
  // 归一化处理
  cv::Mat normalized_img;
  if (input_properties_[0].tensorType == HB_DNN_TENSOR_TYPE_F32) {
    rgb_img.convertTo(normalized_img, CV_32FC3, 1.0 / 255.0);
  } else {
    normalized_img = rgb_img;
  }
  
  // 分配BPU输入内存
  size_t input_size = expected_width * expected_height * expected_channels;
  if (input_properties_[0].tensorType == HB_DNN_TENSOR_TYPE_F32) {
    input_size *= sizeof(float);
  } else {
    input_size *= sizeof(uint8_t);
  }
  
  hbSysMem input_mem;
  int ret = hbSysAllocCachedMem(&input_mem, input_size);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), " 分配BPU输入内存失败！错误码: %d", ret);
    return results;
  }
  
  std::memcpy(input_mem.virAddr, normalized_img.data, input_size);
  hbSysFlushMem(&input_mem, HB_SYS_MEM_CACHE_CLEAN);
  
  // 设置输入张量
  hbDNNTensor input_tensor;
  input_tensor.sysMem[0] = input_mem;
  input_tensor.properties = input_properties_[0];
  
  // 计算输出张量大小
  size_t output_size = 1;
  for (int i = 0; i < output_properties_[0].validShape.numDimensions; i++) {
    output_size *= output_properties_[0].validShape.dimensionSize[i];
  }
  
  if (output_properties_[0].tensorType == HB_DNN_TENSOR_TYPE_F32) {
    output_size *= sizeof(float);
  } else if (output_properties_[0].tensorType == HB_DNN_TENSOR_TYPE_S32) {
    output_size *= sizeof(int32_t);
  } else {
    output_size *= sizeof(uint8_t);
  }
  
  // 分配BPU输出内存
  hbSysMem output_mem;
  ret = hbSysAllocCachedMem(&output_mem, output_size);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), " 分配BPU输出内存失败！错误码: %d", ret);
    hbSysFreeMem(&input_mem);
    return results;
  }
  
  // 设置输出张量
  hbDNNTensor output_tensor;
  output_tensor.sysMem[0] = output_mem;
  output_tensor.properties = output_properties_[0];
  
  // 执行推理
  hbDNNTaskHandle_t task_handle = nullptr;
  hbDNNTensor *output_tensors[] = { &output_tensor };
  hbDNNInferCtrlParam infer_ctrl_param;
  HB_DNN_INITIALIZE_INFER_CTRL_PARAM(&infer_ctrl_param);
  ret = hbDNNInfer(&task_handle, output_tensors, &input_tensor, model_handle_, &infer_ctrl_param);
  
  if (ret == 0 && task_handle) {
    ret = hbDNNWaitTaskDone(task_handle, 0);
    if (ret == 0) {
      hbSysFlushMem(&output_mem, HB_SYS_MEM_CACHE_INVALIDATE);
      
      // 首次推理时打印调试信息
      static bool debug_printed = false;
      if (!debug_printed) {
        debug_print_output_info(output_mem.virAddr, output_properties_[0], output_size);
        debug_printed = true;
      }
      
      results = parse_yolo_output(
        output_mem.virAddr,
        output_properties_[0],
        frame.cols, frame.rows,
        expected_width, expected_height
      );
    } else {
      RCLCPP_ERROR(this->get_logger(), "BPU推理等待失败！错误码: %d", ret);
    }
    hbDNNReleaseTask(task_handle);
  } else {
    RCLCPP_ERROR(this->get_logger(), " BPU推理执行失败！错误码: %d", ret);
  }
  
  // 释放内存
  hbSysFreeMem(&input_mem);
  hbSysFreeMem(&output_mem);
  
  RCLCPP_DEBUG(this->get_logger(), " BPU推理完成，检测到 %zu 个目标", results.size());
  return results;
}

// ==================== 辅助函数 ====================
size_t VisionNode::curl_write_callback(void * contents, size_t size, size_t nmemb, void * userp)
{
  ((std::string*)userp)->append((char*)contents, size * nmemb);
  return size * nmemb;
}

std::string VisionNode::base64_encode(unsigned char const * bytes_to_encode, unsigned int in_len)
{
  std::string ret;
  int i = 0, j = 0;
  unsigned char array_3[3];
  unsigned char array_4[4];
  
  while (in_len--) {
    array_3[i++] = *(bytes_to_encode++);
    if (i == 3) {
      array_4[0] = (array_3[0] & 0xfc) >> 2;
      array_4[1] = ((array_3[0] & 0x03) << 4) + ((array_3[1] & 0xf0) >> 4);
      array_4[2] = ((array_3[1] & 0x0f) << 2) + ((array_3[2] & 0xc0) >> 6);
      array_4[3] = array_3[2] & 0x3f;
      for(i = 0; i < 4; i++) ret += base64_chars[array_4[i]];
      i = 0;
    }
  }
  
  if (i) {
    for(j = i; j < 3; j++) array_3[j] = '\0';
    array_4[0] = (array_3[0] & 0xfc) >> 2;
    array_4[1] = ((array_3[0] & 0x03) << 4) + ((array_3[1] & 0xf0) >> 4);
    array_4[2] = ((array_3[1] & 0x0f) << 2) + ((array_3[2] & 0xc0) >> 6);
    for (j = 0; j < i + 1; j++) ret += base64_chars[array_4[j]];
    while((i++ < 3)) ret += '=';
  }
  return ret;
}

std::string VisionNode::mat_to_base64(const cv::Mat & img)
{
  std::vector<uchar> buf;
  cv::imencode(".jpg", img, buf, {cv::IMWRITE_JPEG_QUALITY, 75});
  return base64_encode(buf.data(), buf.size());
}

// ==================== 千问大模型API调用 ====================
std::string VisionNode::call_qwen_llm_api(const cv::Mat & cropped_img)
{
  std::string base64_str = mat_to_base64(cropped_img);
  std::string data_uri = "data:image/jpeg;base64," + base64_str;

  // 构建遗忘物品字典字符串
  std::string dict_prompt = "";
  for (const auto & item : forgotten_items_dict_) { 
    dict_prompt += "「" + item + "」"; 
  }
  
  // 优化后的提示词
  std::string prompt = R"(
你是一个实验室巡检助手。请分析图中的桌面区域，判断是否存在异常。

【判断规则】
1. 如果图中包含以下常用物品：)" + dict_prompt + R"(，请回复格式：「遗落物品: [物品名称]」
2. 如果图中存在杂乱线缆、垃圾、杂物等不属于上述字典但明显凌乱的物品，请回复：「桌面杂乱」
3. 如果图片内容正常、整洁，没有明显问题，请回复：「无异常」

【重要】
- 只回复上述三种格式之一，不要添加任何解释
- 物品名称使用中文
- 如果有多个物品，只回复最主要的一个
)";

  json request_json = {
    {"model", "qwen-vl-plus"},
    {"messages", json::array({
      {
        {"role", "user"},
        {"content", json::array({
          {{"type", "image_url"}, {"image_url", {{"url", data_uri}}}},
          {{"type", "text"}, {"text", prompt}}
        })}
      }
    })}
  };

  std::string request_body = request_json.dump();
  std::string response_string;
  CURL * curl = curl_easy_init();
  
  if (curl) {
    struct curl_slist * headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    std::string auth_header = "Authorization: Bearer " + qwen_api_key_;
    headers = curl_slist_append(headers, auth_header.c_str());

    curl_easy_setopt(curl, CURLOPT_URL, "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
    curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, curl_write_callback);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);
    curl_easy_setopt(curl, CURLOPT_TIMEOUT, 12L);

    CURLcode res = curl_easy_perform(curl);
    curl_slist_free_all(headers);
    curl_easy_cleanup(curl);
    
    if (res != CURLE_OK) {
      RCLCPP_ERROR(this->get_logger(), " 千问API请求失败: %s", curl_easy_strerror(res));
      return "网络波动: 连云失败";
    }
  }

  try {
    auto res_json = json::parse(response_string);
    if (res_json.contains("output") && res_json["output"].contains("choices") && 
        !res_json["output"]["choices"].empty()) {
      std::string content = res_json["output"]["choices"][0]["message"]["content"];
      // 清理可能的换行和空格
      content.erase(std::remove(content.begin(), content.end(), '\n'), content.end());
      content.erase(std::remove(content.begin(), content.end(), '\r'), content.end());
      return content;
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), " 解析千问响应失败: %s", e.what());
  }
  return "未知杂物";
}

// ==================== 华为云上报（AT+HMPUB 命令） ====================
// 正确格式（一步到位，payload 内联在 AT 命令中）:
//   AT+HMPUB=<session>,"<topic>",<payload_length>,<payload_json>
// 返回 +HMPUB OK 即表示上报成功
// 关键: Topic 字符串必须用 escape_at_string 转义，否则含 $ / 等字符会被 AT 解析器截断
//        payload 是命令最后一个参数，其内部的逗号不会被 AT 解析器当作分隔符
void VisionNode::report_to_huawei_cloud(const std::string & raw_json_payload)
{
  RCLCPP_INFO(this->get_logger(), "执行 L610 硬件连云上报...");

  // 1. 检查是否已连接华为云（连接由巡逻开始时统一建立）
  if (!mqtt_connected_) {
    RCLCPP_ERROR(this->get_logger(), "华为云未连接，上报失败！请先调用 /connect_huawei_cloud 服务");
    return;
  }

  // 2. 构建属性上报 Topic（华为云 IoTDA 属性上报标准路径）
  std::string topic = "$oc/devices/" + device_id_ + "/sys/properties/report";
  size_t payload_len = raw_json_payload.length();

  RCLCPP_INFO(this->get_logger(), "上报Topic: %s, payload=%zu bytes", topic.c_str(), payload_len);

  // 3. AT+HMPUB 一步发布: payload 作为第4个参数直接内联在命令中
  if (uart_fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "串口未打开");
    mqtt_connected_ = false;
    return;
  }

  tcflush(uart_fd_, TCIOFLUSH);

  // 格式: AT+HMPUB=0,"<topic>",<length>,<payload_json>
  // L610 使用 <length> 字段确定 payload 字节数，payload 内部逗号不会被当作分隔符
  // payload 不需引号包裹（L610 不支持，会报 +HMPUB ERR:1）
  std::string pub_cmd = "AT+HMPUB=0,\"" + escape_at_string(topic) + "\","
                        + std::to_string(payload_len) + ","
                        + raw_json_payload + "\r\n";
  
  // 调试: 打印实际 AT 命令（截断前200字符）和 JSON payload
  RCLCPP_INFO(this->get_logger(), "发送AT命令(前200字符): %.200s", pub_cmd.c_str());
  RCLCPP_INFO(this->get_logger(), "JSON payload: %s", raw_json_payload.c_str());
  
  write(uart_fd_, pub_cmd.c_str(), pub_cmd.length());

  // 等待 ACK (+HMPUB OK)
  std::string reply;
  char buf[256];
  int elapsed = 0;
  const int timeout_ms = 10000;
  bool ok = false;

  while (elapsed < timeout_ms) {
    int len = read(uart_fd_, buf, sizeof(buf) - 1);
    if (len > 0) {
      buf[len] = '\0';
      reply += buf;
      // ★ 修复: 匹配 +HMPUB 后紧跟空格/冒号/ERR/OK（区分 AT+HMPUB= 回显）
      //   AT 回显格式: AT+HMPUB=0,...
      //   实际应答格式: +HMPUB OK  或  +HMPUB ERR:N
      if (reply.find("+HMPUB OK") != std::string::npos) {
        ok = true;
        break;
      }
      if (reply.find("+HMPUB ERR") != std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "HMPUB 返回错误, 完整响应: %s", reply.c_str());
        break;
      }
      // 兜底: 如果只收到 ERROR（非 HMPUB 场景）
      if (reply.find("ERROR") != std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "HMPUB 命令被拒绝, 收到: %s", reply.c_str());
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    elapsed += 20;
  }

  if (ok) {
    RCLCPP_INFO(this->get_logger(), "华为云数据上报成功, payload=%zu bytes, 应答: %s", 
                payload_len, reply.c_str());
  } else {
    RCLCPP_WARN(this->get_logger(), "上报ACK超时或失败, 收到: %s", reply.c_str());
    RCLCPP_ERROR(this->get_logger(), "华为云数据上报失败，尝试重连");
    mqtt_connected_ = false;
  }
}

// ==================== 短信发送 ====================
void VisionNode::send_hardware_sms(const std::string & phone_number, const std::string & text_message)
{
  RCLCPP_INFO(this->get_logger(), " 向物理基站发射 AT 短信，接收目标: %s", phone_number.c_str());
  
  send_l610_at_command("AT+CMGF=1", "OK", 1000);
  send_l610_at_command("AT+CSCS=\"GSM\"", "OK", 1000);
  
  std::string cmd = "AT+CMGS=\"" + phone_number + "\"\r\n";
  write(uart_fd_, cmd.c_str(), cmd.length());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  
  std::string sms_payload = text_message + static_cast<char>(26);
  write(uart_fd_, sms_payload.c_str(), sms_payload.length());
  std::this_thread::sleep_for(std::chrono::seconds(2));
}

// ==================== 核心业务：工位巡检处理（根据实际标签） ====================
void VisionNode::handle_inspection(
  const std::shared_ptr<lab_inspection::srv::TriggerInspection::Request> request,
  std::shared_ptr<lab_inspection::srv::TriggerInspection::Response> response)
{
  int station_id = request->station_id;
  RCLCPP_INFO(this->get_logger(), "======>  接收到小车巡航触发服务，工位ID: [%d]", station_id);

  // 从 ROS 图像话题缓存中获取最新帧
  cv::Mat captured_frame;
  {
    std::lock_guard<std::mutex> lock(image_mutex_);
    if (!has_image_ || latest_image_.empty()) {
      response->success = false;
      response->message = "错误: 尚未收到摄像头图像，请确认 originbot hobot_usb_cam 节点已启动并发布到 " + camera_topic_;
      RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
      return;
    }
    captured_frame = latest_image_.clone();
  }
  
  if(captured_frame.empty()) {
    response->success = false;
    response->message = "错误: 物理图像拉取失败";
    return;
  }

  // BPU推理
  std::vector<DetectionResult> bpu_results = execute_bpu_inference(captured_frame);

  bool has_chair_violation = false;      // 椅子未归位（阻塞过道）
  bool has_any_abnormality = false;      // 是否有任何异常
  std::string report_detail = "工位 " + std::to_string(station_id) + " 检测结果: ";
  std::vector<std::string> abnormal_items;  // 异常项列表
  std::vector<cv::Mat> messy_rois;          // 需要大模型研判的杂乱区域
  
  // 设备状态统计（用于上报华为云）
  json device_status = json::object();
  device_status["station_id"] = station_id;
  device_status["timestamp"] = std::time(nullptr);
  device_status["chair"] = "normal";      // 默认正常
  device_status["oscilloscope"] = "UNKNOWN";
  device_status["siggen"] = "UNKNOWN";
  device_status["psu"] = "UNKNOWN";
  device_status["dmm"] = "UNKNOWN";
  device_status["has_mess"] = false;

  // 解析检测结果（根据实际标签）
  for (const auto & det : bpu_results) {
    const ClassInfo* info = get_class_info(det.id);
    
    if (!info) {
      RCLCPP_WARN(this->get_logger(), " 未知类别ID: %d", det.id);
      continue;
    }
    
    RCLCPP_DEBUG(this->get_logger(), 
      " 检测到目标: ID=%d, 名称=%s, 置信度=%.3f, 位置=(%d,%d,%d,%d)",
      det.id, info->name.c_str(), det.score, 
      det.bbox.x, det.bbox.y, det.bbox.width, det.bbox.height);
    
    // 处理椅子未归位（ID=0）- 异常
    if (det.id == 0) {
      has_chair_violation = true;
      has_any_abnormality = true;
      abnormal_items.push_back(" 过道椅子未归位");
      device_status["chair"] = "untidy";
      report_detail += "[ 过道椅子未归位] ";
    }
    // 处理示波器开机（ID=1）- 异常
    else if (det.id == 1) {
      has_any_abnormality = true;
      abnormal_items.push_back(" 示波器未关机");
      device_status["oscilloscope"] = "ON";
      report_detail += "[ 示波器未关机] ";
    }
    // 处理示波器关机（ID=2）- 正常，但上报
    else if (det.id == 2) {
      device_status["oscilloscope"] = "OFF";
      RCLCPP_DEBUG(this->get_logger(), " 示波器已关机");
    }
    // 处理信号发生器开机（ID=3）- 异常
    else if (det.id == 3) {
      has_any_abnormality = true;
      abnormal_items.push_back(" 信号发生器未关机");
      device_status["siggen"] = "ON";
      report_detail += "[ 信号发生器未关机] ";
    }
    // 处理信号发生器关机（ID=4）- 正常，但上报
    else if (det.id == 4) {
      device_status["siggen"] = "OFF";
      RCLCPP_DEBUG(this->get_logger(), " 信号发生器已关机");
    }
    // 处理电源开机（ID=5）- 异常
    else if (det.id == 5) {
      has_any_abnormality = true;
      abnormal_items.push_back(" 稳压电源未关机");
      device_status["psu"] = "ON";
      report_detail += "[ 稳压电源未关机] ";
    }
    // 处理电源关机（ID=6）- 正常，但上报
    else if (det.id == 6) {
      device_status["psu"] = "OFF";
      RCLCPP_DEBUG(this->get_logger(), " 稳压电源已关机");
    }
    // 处理万用表开机（ID=7）- 异常
    else if (det.id == 7) {
      has_any_abnormality = true;
      abnormal_items.push_back(" 万用表未关机");
      device_status["dmm"] = "ON";
      report_detail += "[ 万用表未关机] ";
    }
    // 处理万用表关机（ID=8）- 正常，但上报
    else if (det.id == 8) {
      device_status["dmm"] = "OFF";
      RCLCPP_DEBUG(this->get_logger(), " 万用表已关机");
    }
    // 处理杂乱区域（ID=9）- 异常，需要大模型研判
    else if (det.id == 9) {
      has_any_abnormality = true;
      device_status["has_mess"] = true;
      // 保存ROI区域，稍后统一调用大模型
      cv::Rect safe_bbox = det.bbox & cv::Rect(0, 0, captured_frame.cols, captured_frame.rows);
      if (safe_bbox.width > 10 && safe_bbox.height > 10) {
        messy_rois.push_back(captured_frame(safe_bbox));
      }
    }
  }
  
  // 处理杂乱区域（调用千问大模型）
  std::string messy_judgment;
  for (size_t i = 0; i < messy_rois.size(); i++) {
    if (!messy_rois[i].empty()) {
      std::string qwen_result = call_qwen_llm_api(messy_rois[i]);
      if (!qwen_result.empty() && qwen_result != "未知杂物" && qwen_result != "网络波动: 连云失败") {
        messy_judgment = qwen_result;
        device_status["mess_detail"] = qwen_result;
        report_detail += "[ " + qwen_result + "] ";
      } else if (!qwen_result.empty()) {
        device_status["mess_detail"] = qwen_result;
        report_detail += "[ 检测到杂乱区域] ";
      }
    }
  }
  
  // ==================== 上报华为云（无论是否有异常，所有状态都上报） ====================
  // 华为云 IoTDA 属性上报格式: {"services":[{"service_id":"xxx","properties":{...}}]}
  // 注意: 直连设备不要用外层数组 [{...}]，那是网关批量上报子设备的格式
  json huawei_payload = {
    {"services", json::array({
      {
        {"service_id", "lab_inspection"},
        {"properties", {
          {"station_id", station_id},
          {"chair_status", device_status["chair"]},
          {"oscilloscope", device_status["oscilloscope"]},
          {"siggen", device_status["siggen"]},
          {"psu", device_status["psu"]},
          {"dmm", device_status["dmm"]},
          {"has_mess", device_status["has_mess"]},
          {"has_abnormality", has_any_abnormality || has_chair_violation}
        }}
      }
    })}
  };

  // 如果有杂乱详情，添加到 properties 中
  if (device_status.contains("mess_detail")) {
    huawei_payload["services"][0]["properties"]["mess_detail"] = device_status["mess_detail"];
  }
  
  // 上报华为云
  report_to_huawei_cloud(huawei_payload.dump());
  
  // ==================== 日志记录 ====================
  RCLCPP_INFO(this->get_logger(), 
    " 设备状态上报 - 工位%d: 椅子=%s, 示波器=%s, 信号源=%s, 电源=%s, 万用表=%s, 杂乱=%s, 有异常=%s",
    station_id,
    device_status["chair"].get<std::string>().c_str(),
    device_status["oscilloscope"].get<std::string>().c_str(),
    device_status["siggen"].get<std::string>().c_str(),
    device_status["psu"].get<std::string>().c_str(),
    device_status["dmm"].get<std::string>().c_str(),
    device_status["has_mess"].get<bool>() ? "有" : "无",
    (has_any_abnormality || has_chair_violation) ? "是" : "否");
  
  // 如果没有检测到任何异常
  if (!has_any_abnormality && !has_chair_violation) {
    response->success = true;
    response->message = " 检查完毕，该工位设备均已关机且椅子归位，一切正常。";
    RCLCPP_INFO(this->get_logger(), " 工位 %d 巡检通过，无异常", station_id);
    return;
  }
  
  // ==================== 有异常时执行报警逻辑 ====================
  
  // 构建报警消息
  std::string alert_message;
  if (has_chair_violation) {
    alert_message = "【公共安全告警】实验室过道有椅子挡路，请立即归位！";
  } else {
    alert_message = "【工位告警】检测到异常: ";
    for (size_t i = 0; i < abnormal_items.size(); i++) {
      alert_message += abnormal_items[i];
      if (i < abnormal_items.size() - 1) alert_message += "、";
    }
    if (!messy_judgment.empty() && messy_judgment.find("遗落物品") != std::string::npos) {
      alert_message += "。" + messy_judgment;
    } else if (!messy_rois.empty()) {
      alert_message += "。桌面存在杂乱物品，请整理";
    }
  }
  
  // 短信通知
  if (has_chair_violation) {
    // 椅子未归位：通知公共联系人（station_0）
    auto it = station_contacts_map_.find(0);
    if (it != station_contacts_map_.end()) {
      for (const auto & phone : it->second) {
        send_hardware_sms(phone, alert_message);
        RCLCPP_INFO(this->get_logger(), " 已发送公共告警短信至: %s", phone.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), " 未配置公共联系人(station_0)，无法发送椅子告警");
    }
  } else {
    // 定向通知当前工位负责人
    auto it = station_contacts_map_.find(station_id);
    if (it != station_contacts_map_.end()) {
      for (const auto & phone : it->second) {
        send_hardware_sms(phone, alert_message);
        RCLCPP_INFO(this->get_logger(), " 已发送告警短信至工位%d负责人: %s", station_id, phone.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), " 工位%d未配置联系人，无法发送告警", station_id);
    }
  }
  
  response->success = true;
  response->message = report_detail.empty() ? alert_message : report_detail;
  
  RCLCPP_INFO(this->get_logger(), " 巡检完成: %s", response->message.c_str());
}

// ==================== HMAC-SHA256 实现（华为IoT鉴权用） ====================
} // namespace lab_inspection

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(lab_inspection::VisionNode)

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  // curl_global_init 应在进程级别调用一次，放在 main 中而非节点构造函数
  curl_global_init(CURL_GLOBAL_ALL);

  auto node = std::make_shared<lab_inspection::VisionNode>();

  // 使用 MultiThreadedExecutor：handle_inspection 中有阻塞 CURL/串口 I/O，
  // 单线程 spin 会导致所有回调被阻塞
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  curl_global_cleanup();
  rclcpp::shutdown();
  return 0;
}
