#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <std_msgs/msg/empty.hpp>

class HebiJ6Rotator4 : public rclcpp::Node {
public:
  HebiJ6Rotator4(const rclcpp::NodeOptions& options) : Node("hebi_j6_rotator4", options) {
    // コールバックをマルチスレッドで並行処理できるようにする設定
    callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // Python側の司令塔が出す「/exec_j6_only4」という合図を待つ受信機
    trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/exec_j6_only4", 10, std::bind(&HebiJ6Rotator4::execute_move_callback, this, std::placeholders::_1), sub_options);

    RCLCPP_INFO(this->get_logger(), "J6 Rotator 4 Node Ready. Waiting for /exec_j6_only4...");
  }

private:
  void execute_move_callback(const std_msgs::msg::Empty::SharedPtr) {
    auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "hebi_arm");

    std::vector<double> current_positions = move_group.getCurrentJointValues();
    std::vector<std::string> joint_names = move_group.getJointNames();

    moveit_msgs::msg::RobotTrajectory robot_traj;
    robot_traj.joint_trajectory.joint_names = joint_names;

    double amplitude = 1.0;  // 振幅（ラジアン）
    double time_step = 0.5;  // 反転間隔（秒）
    int j6_idx = 5;          // J6は6番目の関節（インデックス5）
    double start_j6 = current_positions[j6_idx]; // J6の現在位置

    trajectory_msgs::msg::JointTrajectoryPoint p0;
    p0.positions = current_positions;
    p0.time_from_start = rclcpp::Duration::from_seconds(0.0);
    robot_traj.joint_trajectory.points.push_back(p0);

    for (int i = 1; i <= 4; ++i) {
        trajectory_msgs::msg::JointTrajectoryPoint p;
        p.positions = current_positions; 
        p.positions[j6_idx] = start_j6 + ((i % 2 == 1) ? amplitude : -amplitude);
        p.time_from_start = rclcpp::Duration::from_seconds(time_step * i);
        robot_traj.joint_trajectory.points.push_back(p);
    }

    RCLCPP_INFO(this->get_logger(), "MoveItの計画ラグをスキップ！高速交互回転を実行します！");
    move_group.execute(robot_traj);
  }

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
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
