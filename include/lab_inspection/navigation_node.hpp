#ifndef LAB_INSPECTION__NAVIGATION_NODE_HPP_
#define LAB_INSPECTION__NAVIGATION_NODE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "lab_inspection/srv/trigger_inspection.hpp"
#include "std_srvs/srv/trigger.hpp"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
using TriggerInspection = lab_inspection::srv::TriggerInspection;

namespace lab_inspection
{

/**
 * @class NavigationNode
 * @brief 导航节点：控制小车按工位巡航，并呼叫视觉服务
 */
class NavigationNode : public rclcpp::Node
{
public:
  NavigationNode();
  ~NavigationNode();

private:

  struct Workstation
  {
    int id;
    std::string name;
    double x;
    double y;
    double yaw;
    
    Workstation(int id_, const std::string& name_, double x_, double y_, double yaw_)
      : id(id_), name(name_), x(x_), y(y_), yaw(yaw_) {}
  };

  double initial_x_;
  double initial_y_;
  double initial_yaw_;
  std::string frame_id_;
  std::vector<Workstation> workstations_;

  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_action_client_;
  rclcpp::Service<TriggerInspection>::SharedPtr inspection_service_;
  rclcpp::Client<TriggerInspection>::SharedPtr vision_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr skip_station_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr connect_cloud_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr disconnect_cloud_client_;

  bool goal_done_;
  bool goal_succeeded_;
  bool goal_rejected_;  // 区分「目标被拒」(lifecycle未就绪) 与「导航执行失败」
  std::mutex goal_mutex_;
  std::condition_variable goal_cv_;
  
  std::atomic<bool> key_pressed_;
  std::atomic<bool> inspection_running_;
  std::thread inspection_thread_;

  void load_workstations();
  geometry_msgs::msg::Quaternion euler_to_quaternion(double roll, double pitch, double yaw);
  void set_initial_pose();
  geometry_msgs::msg::PoseStamped create_pose_stamped(double x, double y, double yaw = 0.0);
  bool navigate_to_pose(double x, double y, double yaw = 0.0);
  void goal_response_callback(GoalHandleNavigateToPose::SharedPtr goal_handle);
  void feedback_callback(GoalHandleNavigateToPose::SharedPtr, const std::shared_ptr<const NavigateToPose::Feedback> feedback);
  void result_callback(const GoalHandleNavigateToPose::WrappedResult& result);
  void run_inspection();
  void trigger_inspection_callback(const std::shared_ptr<TriggerInspection::Request> request, std::shared_ptr<TriggerInspection::Response> response);
  void skip_station_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                             std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  bool wait_for_vision_or_key(int station_id);

  // 当前机器人位置（从 /amcl_pose 订阅），用于近距离跳过导航
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_pose_sub_;
  void amcl_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg);
};

}  // namespace lab_inspection

#endif  // LAB_INSPECTION__NAVIGATION_NODE_HPP_
