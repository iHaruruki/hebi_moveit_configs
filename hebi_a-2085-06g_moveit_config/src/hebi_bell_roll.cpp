#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/empty.hpp>

class HebiJ6Rotator4 : public rclcpp::Node {
public:
  HebiJ6Rotator4(const rclcpp::NodeOptions& options) : Node("hebi_bell_roll", options) {
    // コールバックをマルチスレッドで並行処理できるようにする設定
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/exec_roll", 10, std::bind(&HebiJ6Rotator4::execute_move_callback, this, std::placeholders::_1), sub_options);
    finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
        "/move_done", 10);

    RCLCPP_INFO(this->get_logger(), "Hebi Arm Mover (Multithreaded + No Wait) started.");
  }

private:
  void execute_move_callback(const std_msgs::msg::Empty::SharedPtr) {
    auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "hebi_arm");

    std::vector<double> current_positions = move_group.getCurrentJointValues();
    std::vector<std::string> joint_names = move_group.getJointNames();

    moveit_msgs::msg::RobotTrajectory robot_traj;
    robot_traj.joint_trajectory.joint_names = joint_names;

    double amplitude = 0.65;
    double time_step = 0.10;
    double start_j6 = 4.730; //current_positions[5];
    RCLCPP_INFO(this->get_logger(), "Current J6 = '%.3f'", start_j6);

    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = current_positions;
    p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
    robot_traj.joint_trajectory.points.push_back(p0);

    trajectory_msgs::msg::JointTrajectoryPoint p1;
    p1.positions = current_positions; 
    p1.positions[5] = current_positions[5] + 0.40;
    p1.time_from_start = rclcpp::Duration::from_seconds(0.12);
    robot_traj.joint_trajectory.points.push_back(p1);
    
    trajectory_msgs::msg::JointTrajectoryPoint p2;
    p2.positions = current_positions; 
    p2.positions[5] = current_positions[5] + 0.80;
    p2.time_from_start = rclcpp::Duration::from_seconds(0.16);
    robot_traj.joint_trajectory.points.push_back(p2);
    
    // Return J6 to its original angle
    trajectory_msgs::msg::JointTrajectoryPoint p_end;
    p_end.positions = current_positions;
    p_end.positions[5] = start_j6;
    p_end.time_from_start = rclcpp::Duration::from_seconds(0.25);
    robot_traj.joint_trajectory.points.push_back(p_end);

    RCLCPP_INFO(this->get_logger(), "MoveItの計画ラグをスキップ！高速交互回転を実行します！");
    move_group.execute(robot_traj);
    
    RCLCPP_INFO(this->get_logger(), "Execute request success!");
    RCLCPP_INFO(this->get_logger(), "===== Publishing /move_done =====");
    finish_pub_->publish(std_msgs::msg::Empty());
  }
  
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  
  auto node = std::make_shared<HebiJ6Rotator4>(node_options);
  
  // マルチスレッドで実行して常に合図を待機
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  
  rclcpp::shutdown();
  return 0;
}
