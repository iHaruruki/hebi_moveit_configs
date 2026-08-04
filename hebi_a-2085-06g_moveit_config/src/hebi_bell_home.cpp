#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>

class MoveToHomeNode : public rclcpp::Node {
public:
    MoveToHomeNode() : Node("move_to_home_node") {
        // /exec_homeトピックを購読
        trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/exec_home", 10, std::bind(&MoveToHomeNode::topic_callback, this, std::placeholders::_1));
        finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/move_done", 10);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /exec_home topic. Waiting for messages...");
    }

private:
    void topic_callback(const std_msgs::msg::Empty::SharedPtr msg) {
    	RCLCPP_INFO(this->get_logger(), "===== Recieved /exec_home =====");
        move_to_home();
    }
    
    void move_to_home() {
        try {
            // MoveGroupInterfaceを作成
            moveit::planning_interface::MoveGroupInterface move_group(
                std::make_shared<rclcpp::Node>("move_group_node"), 
                "hebi_arm"
            );
            move_group.setMaxVelocityScalingFactor(1.0);      // 0.0 - 1.0
            move_group.setMaxAccelerationScalingFactor(0.8);  // 0.0 - 1.0
            
            RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
            RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group.getEndEffectorLink().c_str());
            
            // 現在のロボット状態を取得
            RCLCPP_INFO(this->get_logger(), "Getting current state...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            
            // ホームポジションに設定
            move_group.setNamedTarget("home");
            RCLCPP_INFO(this->get_logger(), "Target set to: home");
            
            // allowed_start_toleranceを緩和する
            move_group.setPlanningTime(10.0);
            
            // プランを作成
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
            if (success) {
            // Estimated execution time from MoveIt trajectory
                auto result = move_group.execute(my_plan);

                if (result == moveit::core::MoveItErrorCode::SUCCESS)
                {
                    RCLCPP_INFO(this->get_logger(), "Execute request success!");
                    RCLCPP_INFO(this->get_logger(), "===== Publishing /move_done =====");
                    finish_pub_->publish(std_msgs::msg::Empty());
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Execution failed!");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create a motion plan.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToHomeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
