#ifndef LAB_INSPECTION__VISION_NODE_HPP_
#define LAB_INSPECTION__VISION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <algorithm>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "opencv2/opencv.hpp"

// ROS 图像消息与 cv_bridge 转换
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>

// 地瓜 RDK 平台 BPU 原生底层推理加速及内存分配库
#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"

// 自定义工位巡检接口服务
#include "lab_inspection/srv/trigger_inspection.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace lab_inspection
{

struct DetectionResult {
  int id;           // 类别ID
  float score;      // 置信度
  cv::Rect bbox;    // 边界框
};

class VisionNode : public rclcpp::Node
{
public:
  explicit VisionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~VisionNode();

private:
  // ==================== 1. 硬件级连续初始化 ====================
  void init_bpu_model();
  void init_l610_uart();
  void init_camera_subscription();  // 订阅 originbot hobot_usb_cam 发布的 ROS 图像话题
  void load_contacts_from_yaml();   // 动态读取 station_contacts 参数

  // ==================== 2. 核心真实业务处理逻辑 ====================
  void handle_inspection(
    const std::shared_ptr<lab_inspection::srv::TriggerInspection::Request> request,
    std::shared_ptr<lab_inspection::srv::TriggerInspection::Response> response);

  /// @brief ROS 图像话题回调：将 hobot_usb_cam 发布的图像缓存到 latest_image_
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::vector<DetectionResult> execute_bpu_inference(const cv::Mat & frame);
  std::string call_qwen_llm_api(const cv::Mat & cropped_img);
  bool send_l610_at_command(const std::string & cmd, const std::string & expected_reply = "OK", int timeout_ms = 2000);
  bool send_l610_at_command(const std::string & cmd, const std::string & expected_reply, int timeout_ms, std::string* out_reply);
  void report_to_huawei_cloud(const std::string & raw_json_payload);
  bool connect_huawei_cloud();
  void disconnect_huawei_cloud();
  void send_hardware_sms(const std::string & phone_number, const std::string & text_message);

  // ==================== 华为云连接管理服务回调 ====================
  void connect_cloud_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void disconnect_cloud_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ==================== 3. BPU输出解析辅助函数 ====================
  std::vector<DetectionResult> parse_yolo_output(
    void* output_data,
    const hbDNNTensorProperties& properties,
    int original_width, int original_height,
    int model_width, int model_height);
  
  std::vector<DetectionResult> apply_nms(
    std::vector<DetectionResult>& detections, 
    float nms_threshold);
  
  float compute_iou(const cv::Rect& box1, const cv::Rect& box2);
  
  // 调试函数：打印输出张量信息
  void debug_print_output_info(void* output_data, const hbDNNTensorProperties& properties, size_t output_size);

  // ==================== 4. 底层数据流转换与网络回调 ====================
  std::string mat_to_base64(const cv::Mat & img);
  std::string base64_encode(unsigned char const * bytes_to_encode, unsigned int in_len);
  static size_t curl_write_callback(void * contents, size_t size, size_t nmemb, void * userp);

  // ==================== 5. 物理硬件与通信句柄 ====================
  rclcpp::Service<lab_inspection::srv::TriggerInspection>::SharedPtr inspection_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr connect_cloud_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disconnect_cloud_service_;
  
  // 摄像头：通过订阅 originbot hobot_usb_cam 发布的 ROS 话题获取图像
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  cv::Mat latest_image_;
  std::mutex image_mutex_;
  bool has_image_ = false;
  
  hbPackedDNNHandle_t packed_model_ = nullptr;
  hbDNNHandle_t model_handle_ = nullptr;
  
  // BPU模型输入输出属性（新增）
  hbDNNTensorProperties* input_properties_ = nullptr;
  hbDNNTensorProperties* output_properties_ = nullptr;
  int input_count_ = 0;
  int output_count_ = 0;
  
  int uart_fd_ = -1;                   

  // ==================== 6. 严格对应 yaml 的物理参数变量 ====================
  std::string model_path_;             
  double conf_threshold_;              
  double io_threshold_;                
  std::string camera_topic_;           // 摄像头 ROS 话题名（如 /image_raw）
  int camera_queue_size_ = 5;          // 订阅队列大小
  std::string huawei_mqtt_server_;     
  int huawei_mqtt_port_;              
  std::string qwen_api_key_;           
  std::string uart_device_;            
  int uart_baudrate_;                  
  std::vector<std::string> forgotten_items_dict_; 
  
  // 完美映射 yaml 中的嵌套手机号字典
  std::map<int, std::vector<std::string>> station_contacts_map_;
  
  std::mutex bpu_mutex_;
  bool mqtt_connected_ = false;
  int mqtt_msg_id_ = 1;              // MQTT 消息ID递增
  std::string device_id_;            // 华为云设备ID
  std::string device_secret_;        // 华为云设备密钥


};

}  // namespace lab_inspection

#endif  // LAB_INSPECTION__VISION_NODE_HPP_