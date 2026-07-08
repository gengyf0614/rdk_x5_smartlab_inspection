#include <array>
#include <chrono>
#include "lab_inspection/navigation_node.hpp"

namespace lab_inspection
{

static const std::chrono::seconds kNavActionServerTimeout(120);
static const std::chrono::seconds kVisionServiceTimeout(5);
static const std::chrono::seconds kVisionResponseTimeout(30);

NavigationNode::~NavigationNode()
{
  key_pressed_.store(true);
  if (inspection_thread_.joinable()) {
    inspection_thread_.join();
    RCLCPP_INFO(get_logger(), "巡检线程已终止");
  }
}

NavigationNode::NavigationNode()
  : Node("navigation_node"),
    goal_done_(false),
    goal_succeeded_(false),
    goal_rejected_(false),
    key_pressed_(false),
    inspection_running_(false)
{
  RCLCPP_INFO(get_logger(), "导航节点已启动");

  // 声明参数
  this->declare_parameter<double>("initial_x", 0.0);
  this->declare_parameter<double>("initial_y", 0.0);
  this->declare_parameter<double>("initial_yaw", 0.0);
  this->declare_parameter<std::string>("frame_id", "map");

  // 获取参数
  this->get_parameter("initial_x", initial_x_);
  this->get_parameter("initial_y", initial_y_);
  this->get_parameter("initial_yaw", initial_yaw_);
  this->get_parameter("frame_id", frame_id_);

  // 加载工位配置
  load_workstations();

  RCLCPP_INFO(get_logger(), "初始位置: x=%f, y=%f, yaw=%f", 
              initial_x_, initial_y_, initial_yaw_);
  RCLCPP_INFO(get_logger(), "工位数: %zu", workstations_.size());

  // 创建初始位姿发布者
  initial_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);

  // 创建导航Action客户端（不阻塞等待）
  nav_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "/navigate_to_pose");

  // 创建视觉检测服务客户端（不阻塞等待）
  vision_client_ = this->create_client<TriggerInspection>("/trigger_inspection");

  // 创建自检服务
  inspection_service_ = this->create_service<TriggerInspection>(
      "/start_inspection",
      std::bind(&NavigationNode::trigger_inspection_callback, this, 
                std::placeholders::_1, std::placeholders::_2));

  // 创建跳过工位服务（替代键盘Enter）
  skip_station_service_ = this->create_service<std_srvs::srv::Trigger>(
      "/skip_station",
      std::bind(&NavigationNode::skip_station_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // 创建华为云连接/断开客户端
  connect_cloud_client_ = this->create_client<std_srvs::srv::Trigger>("/connect_huawei_cloud");
  disconnect_cloud_client_ = this->create_client<std_srvs::srv::Trigger>("/disconnect_huawei_cloud");

  RCLCPP_INFO(get_logger(), "导航节点初始化完成");
  RCLCPP_INFO(get_logger(), "等待服务调用...");
  RCLCPP_INFO(get_logger(), "提示: 导航服务和视觉服务可稍后启动");

  // 初始位姿由 AMCL 的 set_initial_pose:true 在激活时自动设置，无需 navigation_node 再发
}

void NavigationNode::load_workstations()
{
  // 硬编码工位配置
  // 格式：(工位ID, 工位名称, X坐标, Y坐标, 朝向(弧度))
  workstations_.emplace_back(1, "工位0", 3.0, 0.5,1.57);    
  workstations_.emplace_back(2, "工位1", 3.1, 1.56, -3.15);   
  workstations_.emplace_back(3, "工位2", 3.1, 4.49, -3.15);   
  workstations_.emplace_back(4, "工位3", 3.1, 5.63, -3.15);   

  // 打印工位信息
  RCLCPP_INFO(get_logger(), "共加载 %zu 个工位", workstations_.size());
  for (const auto& ws : workstations_) {
    RCLCPP_INFO(get_logger(), "工位%d: %s, 位置: (%f, %f), 朝向: %.1f°", 
                ws.id, ws.name.c_str(), ws.x, ws.y, ws.yaw * 180.0 / M_PI);
  }
}

geometry_msgs::msg::Quaternion NavigationNode::euler_to_quaternion(double roll, double pitch, double yaw)
{
  geometry_msgs::msg::Quaternion q;
  double cy = cos(yaw * 0.5);
  double sy = sin(yaw * 0.5);
  double cp = cos(pitch * 0.5);
  double sp = sin(pitch * 0.5);
  double cr = cos(roll * 0.5);
  double sr = sin(roll * 0.5);

  q.w = cr * cp * cy + sr * sp * sy;
  q.x = sr * cp * cy - cr * sp * sy;
  q.y = cr * sp * cy + sr * cp * sy;
  q.z = cr * cp * sy - sr * sp * cy;

  return q;
}

void NavigationNode::set_initial_pose()
{
  RCLCPP_INFO(get_logger(), "设置机器人初始位姿...");

  geometry_msgs::msg::PoseWithCovarianceStamped msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = this->get_clock()->now();

  msg.pose.pose.position.x = initial_x_;
  msg.pose.pose.position.y = initial_y_;
  msg.pose.pose.position.z = 0.0;
  msg.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, initial_yaw_);

  std::array<double, 36> covariance = {
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0685389192
  };
  msg.pose.covariance = covariance;

  initial_pose_pub_->publish(msg);
  RCLCPP_INFO(get_logger(), "初始位姿已设置: (%f, %f, %f)", 
              initial_x_, initial_y_, initial_yaw_);

}

geometry_msgs::msg::PoseStamped NavigationNode::create_pose_stamped(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id_;
  pose.header.stamp = this->get_clock()->now();  // 使用当前实时时间戳，避免 TF 外推错误

  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = 0.0;
  pose.pose.orientation = euler_to_quaternion(0.0, 0.0, yaw);

  return pose;
}

void NavigationNode::goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "目标点被拒绝！");
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_rejected_ = true;
      goal_done_ = true;
      goal_succeeded_ = false;
    }
    goal_cv_.notify_one();
  } else {
    RCLCPP_INFO(get_logger(), "目标点已被接受，开始导航...");
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_rejected_ = false;
    }
  }
}

void NavigationNode::feedback_callback(GoalHandleNavigateToPose::SharedPtr,
                                      const std::shared_ptr<const NavigateToPose::Feedback>)
{
  // 可以在这里处理导航过程中的反馈信息
}

void NavigationNode::result_callback(const GoalHandleNavigateToPose::WrappedResult& result)
{
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    goal_rejected_ = false;
    goal_done_ = true;
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(get_logger(), "成功到达目标点！");
      goal_succeeded_ = true;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_WARN(get_logger(), "导航被中止！");
      goal_succeeded_ = false;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(get_logger(), "导航被取消！");
      goal_succeeded_ = false;
      break;
    default:
      RCLCPP_WARN(get_logger(), "导航未成功完成，返回码: %d", static_cast<int>(result.code));
      goal_succeeded_ = false;
      break;
    }
  }
  goal_cv_.notify_one();
}

bool NavigationNode::navigate_to_pose(double x, double y, double yaw)
{
  // 检查导航服务是否可用
  if (!nav_action_client_->wait_for_action_server(kNavActionServerTimeout)) {
    RCLCPP_ERROR(get_logger(), "导航服务不可用！请确保已启动Nav2导航功能");
    return false;
  }

  // 重试机制：当目标被拒绝时（生命周期节点尚未激活），等待后重试
  // 若目标被接受但导航执行失败，则不重试直接返回
  const int kMaxRetries = 20;
  const auto kRetryDelay = std::chrono::seconds(3);

  for (int retry = 0; retry < kMaxRetries; ++retry) {
    geometry_msgs::msg::PoseStamped goal_pose = create_pose_stamped(x, y, yaw);

    if (retry == 0) {
      RCLCPP_INFO(get_logger(), "发送目标点: (%f, %f), 朝向: %.1f°", 
                  x, y, yaw * 180.0 / M_PI);
    } else {
      RCLCPP_INFO(get_logger(), "重试发送目标点 (%d/%d): (%f, %f), 朝向: %.1f°",
                  retry + 1, kMaxRetries, x, y, yaw * 180.0 / M_PI);
    }

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      goal_done_ = false;
      goal_succeeded_ = false;
      goal_rejected_ = false;
    }

    auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigationNode::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigationNode::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(&NavigationNode::result_callback, this, std::placeholders::_1);

    nav_action_client_->async_send_goal(goal_msg, send_goal_options);

    // 使用条件变量等待，避免与MultiThreadedExecutor冲突
    {
      std::unique_lock<std::mutex> lock(goal_mutex_);
      goal_cv_.wait(lock, [this]() { return goal_done_ || !rclcpp::ok(); });
    }

    // 检查结果
    if (goal_succeeded_) {
      return true;
    }

    // 如果是被拒绝（生命周期未就绪），重试；否则（导航执行失败）直接返回
    if (!goal_rejected_) {
      RCLCPP_ERROR(get_logger(), "导航执行失败（非拒绝），放弃重试");
      return false;
    }

    // 目标被拒绝，等待后重试
    if (retry < kMaxRetries - 1) {
      RCLCPP_WARN(get_logger(), "导航目标被拒绝（Nav2生命周期节点可能尚未激活），"
                  "%lds 后重试 (%d/%d)...",
                  kRetryDelay.count(), retry + 1, kMaxRetries);
      rclcpp::sleep_for(kRetryDelay);
    }
  }

  RCLCPP_ERROR(get_logger(), "导航目标重试 %d 次后仍被拒绝，放弃", kMaxRetries);
  return false;
}

void NavigationNode::skip_station_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  key_pressed_.store(true);
  response->success = true;
  response->message = "已跳过当前工位，继续前往下一个工位";
  RCLCPP_INFO(get_logger(), "收到跳过工位请求，继续前往下一个工位");
}


bool NavigationNode::wait_for_vision_or_key(int station_id)
{
  // 重置按键状态
  key_pressed_.store(false);

  // 等待视觉服务可用
  if (!vision_client_->wait_for_service(kVisionServiceTimeout)) {
    RCLCPP_WARN(get_logger(), "视觉服务不可用，等待 %lds 后自动继续...", kVisionServiceTimeout.count());
    auto start_time = std::chrono::steady_clock::now();
    while (rclcpp::ok() && !key_pressed_.load()) {
      if (std::chrono::steady_clock::now() - start_time > kVisionResponseTimeout) {
        RCLCPP_WARN(get_logger(), "视觉服务长时间不可用，自动跳过视觉检测");
        return false;
      }
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  } else {
    auto request = std::make_shared<TriggerInspection::Request>();
    request->station_id = station_id;
    RCLCPP_INFO(get_logger(), "调用视觉检测服务，工位: %d", station_id);

    // 异步发送请求
    auto future = vision_client_->async_send_request(request);
    auto start_time = std::chrono::steady_clock::now();

    // 等待视觉响应或键盘输入
    while (rclcpp::ok() && !key_pressed_.load()) {
      if (future.wait_for(std::chrono::milliseconds(100)) == std::future_status::ready) {
        try {
          auto response = future.get();
          RCLCPP_INFO(get_logger(), "视觉检测完成，结果: %s", 
                      response->success ? "成功" : "失败");
          return response->success;
        } catch (...) {
          RCLCPP_ERROR(get_logger(), "获取视觉检测结果失败");
          break;
        }
      }

      if (std::chrono::steady_clock::now() - start_time > kVisionResponseTimeout) {
        RCLCPP_WARN(get_logger(), "视觉响应超时 %lds，继续前往下一个工位", kVisionResponseTimeout.count());
        return false;
      }

      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // 如果是键盘输入触发
  if (key_pressed_.load()) {
      RCLCPP_INFO(get_logger(), "通过服务调用跳过视觉检测，继续前往下一个工位");
    return true;
  }

  return false;
}

void NavigationNode::run_inspection()
{
  // ==================== 巡逻开始：连接华为云 ====================
  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "步骤0: 连接华为云 IoT 平台");
  RCLCPP_INFO(get_logger(), "==================================================");
  if (connect_cloud_client_->wait_for_service(std::chrono::seconds(10))) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = connect_cloud_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(25)) == std::future_status::ready) {
      auto res = future.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "华为云连接成功: %s", res->message.c_str());
      } else {
        RCLCPP_ERROR(get_logger(), "华为云连接失败: %s，巡逻将继续但数据无法上报", res->message.c_str());
      }
    } else {
      RCLCPP_ERROR(get_logger(), "华为云连接超时，巡逻将继续但数据无法上报");
    }
  } else {
    RCLCPP_ERROR(get_logger(), "华为云连接服务不可用，巡逻将继续但数据无法上报");
  }

  // 设置初始位姿
  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "步骤1: 设置机器人初始位姿");
  RCLCPP_INFO(get_logger(), "==================================================");
  set_initial_pose();

  rclcpp::sleep_for(std::chrono::seconds(2));

  // 依次导航到各个工位
  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "步骤2: 开始工位巡航");
  RCLCPP_INFO(get_logger(), "==================================================");

  for (size_t i = 0; i < workstations_.size(); ++i) {
    const auto& ws = workstations_[i];
    RCLCPP_INFO(get_logger(), "--- 工位%d/%zu: %s, 位置: (%f, %f), 朝向: %.1f° ---", 
                ws.id, workstations_.size(), ws.name.c_str(), ws.x, ws.y, ws.yaw * 180.0 / M_PI);

    // 导航到当前工位
    bool nav_success = navigate_to_pose(ws.x, ws.y, ws.yaw);

    if (nav_success) {
      RCLCPP_INFO(get_logger(), "已到达工位%d", ws.id);

      // 发送信号给视觉模块，等待视觉检测完成或键盘输入
      RCLCPP_INFO(get_logger(), "发送信号给视觉模块，等待检测完成...");
      RCLCPP_INFO(get_logger(), "提示: 可通过 /skip_station 服务跳过并继续前往下一个工位");
      bool vision_success = wait_for_vision_or_key(ws.id);

      if (vision_success) {
        RCLCPP_INFO(get_logger(), "准备前往下一个工位");
      } else {
        RCLCPP_WARN(get_logger(), "视觉检测失败，继续前往下一个工位");
      }
    } else {
      RCLCPP_ERROR(get_logger(), "无法到达工位%d，跳过该工位", ws.id);
    }
  }

  // 返回起始位置
  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "所有工位巡航完成！");
  RCLCPP_INFO(get_logger(), "==================================================");

  RCLCPP_INFO(get_logger(), "返回起始位置...");
  navigate_to_pose(initial_x_, initial_y_, initial_yaw_);

  // ==================== 巡逻结束：断开华为云 ====================
  RCLCPP_INFO(get_logger(), "==================================================");
  RCLCPP_INFO(get_logger(), "步骤3: 断开华为云 IoT 连接");
  RCLCPP_INFO(get_logger(), "==================================================");
  if (disconnect_cloud_client_->wait_for_service(std::chrono::seconds(5))) {
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    auto future = disconnect_cloud_client_->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(10)) == std::future_status::ready) {
      auto res = future.get();
      RCLCPP_INFO(get_logger(), "华为云断开: %s", res->message.c_str());
    } else {
      RCLCPP_WARN(get_logger(), "华为云断开请求超时");
    }
  } else {
    RCLCPP_WARN(get_logger(), "华为云断开服务不可用");
  }

  RCLCPP_INFO(get_logger(), "巡航任务全部完成！");
}

void NavigationNode::trigger_inspection_callback(
    const std::shared_ptr<TriggerInspection::Request> request,
    std::shared_ptr<TriggerInspection::Response> response)
{
  (void)request;  // 未使用参数

  // 防止重复触发巡航
  if (inspection_running_.load()) {
    RCLCPP_WARN(get_logger(), "已有巡航任务正在执行，忽略重复请求");
    response->success = false;
    response->message = "已有巡航任务正在执行";
    return;
  }

  RCLCPP_INFO(get_logger(), "收到巡航请求，将在后台执行工位巡检...");

  // 在独立线程中异步执行巡航，避免阻塞 spin 线程
  inspection_running_.store(true);
  if (inspection_thread_.joinable()) {
    inspection_thread_.join();
  }
  inspection_thread_ = std::thread([this]() {
    run_inspection();
    inspection_running_.store(false);
    RCLCPP_INFO(get_logger(), "巡航任务线程结束");
  });

  response->success = true;
  response->message = "巡航任务已接受，正在执行";
}

}  // namespace lab_inspection

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lab_inspection::NavigationNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
