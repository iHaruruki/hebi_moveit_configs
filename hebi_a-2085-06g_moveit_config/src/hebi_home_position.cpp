#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/empty.hpp>

class HomeMover : public rclcpp::Node {
public:
    HomeMover() : Node("hebi_home_position") {
        // Python側の pub_home.publish(Empty()) を待ち受ける
        subscription_ = this->create_subscription<std_msgs::msg::Empty>(
            "/exec_home", 10, std::bind(&HomeMover::home_callback, this, std::placeholders::_1));
        finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/move_done", 10);
        
        RCLCPP_INFO(this->get_logger(), "Home position node ready. Waiting for signal on /exec_home...");
    }

private:
    void home_callback(const std_msgs::msg::Empty::SharedPtr msg) {
        (void)msg;
        RCLCPP_INFO(this->get_logger(), "Signal received! Moving to home position...");

        // MoveItの設定
        auto move_group = moveit::planning_interface::MoveGroupInterface(shared_from_this(), "hebi_arm");
        move_group.setMaxVelocityScalingFactor(0.8);      // 0.0 - 1.0
        move_group.setMaxAccelerationScalingFactor(0.8);  // 0.0 - 1.0
        
        // SRDFで設定した "home" への移動
        move_group.setNamedTarget("home");

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
            move_group.execute(my_plan);
            RCLCPP_INFO(this->get_logger(), "Arrived at home position.");
            finish_pub_->publish(std_msgs::msg::Empty());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to plan motion to home.");
        }
    }
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HomeMover>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
