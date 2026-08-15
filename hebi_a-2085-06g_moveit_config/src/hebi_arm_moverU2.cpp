#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class HebiArmMover : public rclcpp::Node {
public:
  HebiArmMover(const rclcpp::NodeOptions& options) : Node("hebi_arm_moverU2", options) {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    this->declare_parameter<std::string>("target_frame", "teddy bear");
    
    // コールバックをマルチスレッドで並行処理できるようにする設定
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/exec_moveU2", 10, std::bind(&HebiArmMover::execute_move_callback, this, std::placeholders::_1), sub_options);
    finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/move_done", 10);
    saved_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
        "/saved_pose2", 10);
        
    RCLCPP_INFO(this->get_logger(), "Hebi Arm Mover (Multithreaded + No Wait) started.");
  }
  
private:
  void execute_move_callback(const std_msgs::msg::Empty::SharedPtr) { 
    auto move_group_interface = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "hebi_arm");
    move_group_interface.setMaxVelocityScalingFactor(0.8);      // 0.0 - 1.0
    move_group_interface.setMaxAccelerationScalingFactor(0.8);  // 0.0 - 1.0
  
    RCLCPP_INFO(this->get_logger(), "Fetching current joint values...");
    
    // 時間のズレによるエラーが発生しないよう、現在の角度を即座に取得
    std::vector<double> current_joint_values = move_group_interface.getCurrentJointValues();
    
    if (current_joint_values.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to get current joint values. Is the robot running?");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Current joint states received successfully.");
    
    // J6_wrist3 の角度（インデックス5）を保存
    double current_j6_angle = current_joint_values[5];

    move_group_interface.setWorkspace(-2.0, -2.0, -2.0, 2.0, 2.0, 2.0);
  
    const std::string ee_link = "end_effector_1/output";
    move_group_interface.setEndEffectorLink(ee_link);
    
    const std::string target_frame = this->get_parameter("target_frame").as_string();
    
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform("base_link", target_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(), "Could not find TF '%s': %s", target_frame.c_str(), ex.what());
      return;
    }
    
    double offset_x = 0.03;
    double offset_y = 0.01; 
    double offset_z = 0.06; 
    
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp = this->now();
    
    target_pose.pose.position.x = transform.transform.translation.x + offset_x;
    target_pose.pose.position.y = transform.transform.translation.y + offset_y;
    target_pose.pose.position.z = transform.transform.translation.z + offset_z;
  
    tf2::Quaternion q;
    q.setRPY(3.121, 0.006, 1.426);
    target_pose.pose.orientation = tf2::toMsg(q);  
  
    // IKを解くための空のロボット状態を作成
    moveit::core::RobotStatePtr kinematic_state = std::make_shared<moveit::core::RobotState>(move_group_interface.getRobotModel());
    const moveit::core::JointModelGroup* joint_model_group = kinematic_state->getJointModelGroup("hebi_arm");
    
    // 取得した現在の角度をセット
    kinematic_state->setJointGroupPositions(joint_model_group, current_joint_values);

    // IKを計算
    bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose.pose, 0.1);

    if (found_ik) {
      std::vector<double> target_joint_values;
      kinematic_state->copyJointGroupPositions(joint_model_group, target_joint_values);
      
      // J6の角度を現在の角度で上書き
      target_joint_values[5] = current_j6_angle;

      move_group_interface.setJointValueTarget(target_joint_values);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
      
      if (success){
        auto result = move_group_interface.execute(my_plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {   
            auto lifted_pose = move_group_interface.getCurrentPose();
            saved_pose_pub_->publish(lifted_pose);
              
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr saved_pose_pub_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
};
  
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<HebiArmMover>(node_options);
  
  // マルチスレッドで実行
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
