#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/empty.hpp>
#include <vector>

class HebiJ6Rotator : public rclcpp::Node {
public:
  HebiJ6Rotator() : Node("hebi_j6_rotator2") {}
  
  void initialize() {
  
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), "hebi_arm");
    
    // ==================================================
    // 【ここに追加】速度と加速度のスケーリング（0.0 〜 1.0）
    // ==================================================
    // 例：最高速度を limits.yaml の設定値の 50% にする
    move_group_->setMaxVelocityScalingFactor(0.1); 
    
    // 例：最大加速度を limits.yaml の設定値の 50% にする
    move_group_->setMaxAccelerationScalingFactor(0.1);
    // ==================================================
    
    RCLCPP_INFO(this->get_logger(), "==============================");
    
    RCLCPP_INFO(this->get_logger(), "Planning Group: %s", move_group_->getName().c_str());
    
    RCLCPP_INFO(this->get_logger(), "Planning Frame: %s", move_group_->getPlanningFrame().c_str());  
    
    const std::vector<std::string> &joint_names = move_group_->getJointNames();
    RCLCPP_INFO(this->get_logger(), "Joints count: %zu", joint_names.size());
    for (size_t i = 0; i < joint_names.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), " - Joint [%zu]: %s", i, joint_names[i].c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "==============================");
    
    callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::Reentrant
    );
    
    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;
    
    subscription_ = this->create_subscription<std_msgs::msg::Empty>( 
        "/exec_j6_only2", 10, std::bind(&HebiJ6Rotator::rotate_j6_callback, this, std::placeholders::_1), sub_opt);
    finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/move_done", 10);
    RCLCPP_INFO(this->get_logger(), "J6 Rotator Node Ready. Waiting for /exec_j6_only...");
  }
  
private:

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  void rotate_j6_callback(const std_msgs::msg::Empty::SharedPtr msg) {
  
    (void)msg;
    RCLCPP_INFO(this->get_logger(), "get the signal.");  
    
    move_group_->setStartStateToCurrentState();
    std::vector<double> joint_group_positions = move_group_->getCurrentJointValues();
    
    if (joint_group_positions.empty()) {
      RCLCPP_ERROR(this->get_logger(), "joint_group_positions is empty");
      return;
    } 
    
    joint_group_positions[5] += 6.50;
    
    move_group_->setJointValueTarget(joint_group_positions);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    
    if (success){
      auto result = move_group_->execute(my_plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {   
            RCLCPP_INFO(this->get_logger(), "Execute request success!");
            RCLCPP_INFO(this->get_logger(), "===== Publishing /move_done =====");
            finish_pub_->publish(std_msgs::msg::Empty());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Execution failed!");
        }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Planning failed.");     
    }
  }
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
};
  
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HebiJ6Rotator>();
  node->initialize();
  
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
