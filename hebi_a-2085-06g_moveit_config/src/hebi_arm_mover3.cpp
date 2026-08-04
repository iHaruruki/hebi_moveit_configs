#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp> // ← 追加：座標受信用の型
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/empty.hpp>

class HebiArmMover3 : public rclcpp::Node {
public:
  HebiArmMover3(const rclcpp::NodeOptions& options) : Node("hebi_arm_mover3", options) {
    
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    
    // mover2から送られてくる座標を受信して保存するサブスクライバ
    point_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
        "/detected_object_point", 10,
        [this](const geometry_msgs::msg::Point::SharedPtr msg) {
            saved_point_ = *msg;
            point_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Received coordinates from mover2: x=%.3f, y=%.3f, z=%.3f", msg->x, msg->y, msg->z);
        }, sub_options);

    // 実行トリガー
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/exec_move3", 10, std::bind(&HebiArmMover3::execute_move_callback, this, std::placeholders::_1), sub_options);
    finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/move_done", 10);
    
    RCLCPP_INFO(this->get_logger(), "Hebi Arm Mover 3 (Receiver) started.");
  }
  
private:
  void execute_move_callback(const std_msgs::msg::Empty::SharedPtr) { 
    // mover2から座標を受け取っているかチェック
    if (!point_received_) {
        RCLCPP_ERROR(this->get_logger(), "Cannot move: No coordinates received from mover2 yet!");
        return;
    }

    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "hebi_arm");
    move_group_interface.setMaxVelocityScalingFactor(0.8);      // 0.0 - 1.0
    move_group_interface.setMaxAccelerationScalingFactor(0.8);  // 0.0 - 1.0
  
    std::vector<double> current_joint_values = move_group_interface.getCurrentJointValues();
    if (current_joint_values.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current joint values.");
        return;
    }
    double current_j6_angle = current_joint_values[5];

    move_group_interface.setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
    const std::string ee_link = "end_effector_1/output";
    move_group_interface.setEndEffectorLink(ee_link);
    
    // mover3 特有のオフセット値（Zを上げて持ち上げる等）
    double offset_x = 0.00;
    double offset_y = -0.09; 
    double offset_z = -0.03; 
  
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = this->now();
    
    // ==========================================
    // TFではなく、保存しておいた mover2 の座標を使う
    // ==========================================
    target_pose.pose.position.x = saved_point_.x + offset_x;
    target_pose.pose.position.y = saved_point_.y + offset_y;
    target_pose.pose.position.z = saved_point_.z + offset_z;
  
    tf2::Quaternion q;
    q.setRPY(-1.58, -1.43, -0.16); // RPYは元のまま
    target_pose.pose.orientation = tf2::toMsg(q);  
  
    RCLCPP_INFO(this->get_logger(), "Goal based on mover2 point: x=%.3f, y=%.3f, z=%.3f", 
                target_pose.pose.position.x, target_pose.pose.position.y, target_pose.pose.position.z);

    moveit::core::RobotStatePtr kinematic_state = std::make_shared<moveit::core::RobotState>(move_group_interface.getRobotModel());
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup("hebi_arm");
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

    bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose.pose, 0.1);

    if (found_ik) {
      std::vector<double> target_joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, target_joint_values);
      target_joint_values[5] = current_j6_angle; // J6固定
      move_group_interface.setJointValueTarget(target_joint_values);
      
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      if (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS){
        auto result = move_group_interface.execute(my_plan);

          if (result == moveit::core::MoveItErrorCode::SUCCESS)
          {   
              RCLCPP_INFO(this->get_logger(), "Execute request success!");
              RCLCPP_INFO(this->get_logger(), "===== Publishing /move_done =====");
              finish_pub_->publish(std_msgs::msg::Empty());
          } else {
              RCLCPP_ERROR(this->get_logger(), "Execution failed!");
          }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Planning failed with overwritten J6.");      
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "IK Failed.");
    }
  }
  
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr point_sub_; // ← 追加
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  
  geometry_msgs::msg::Point saved_point_; // ← 座標を保存する変数
  bool point_received_ = false;           // ← 受け取ったかどうかのフラグ
};
  
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto node = std::make_shared<HebiArmMover3>(node_options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
