#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <std_msgs/msg/string.hpp>

class MoveToHomeNode : public rclcpp::Node {
public:
    MoveToHomeNode() : Node("move_to_home_node") {
        // /exec_homeトピックを購読
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/exec_home",
            10,
            std::bind(&MoveToHomeNode::topic_callback, this, std::placeholders::_1)
        );
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to /exec_home topic. Waiting for messages...");
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received message: '%s'", msg->data.c_str());
        move_to_home();
    }
    
    void move_to_home() {
        try {
            // MoveGroupInterfaceを作成
            moveit::planning_interface::MoveGroupInterface move_group(
                std::make_shared<rclcpp::Node>("move_group_node"), 
                "hebi_arm"
            );
            
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
                RCLCPP_INFO(this->get_logger(), "Plan created successfully!");
                
                // 状態を更新してから実行
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                
                // プランを実行
                bool executed = (move_group.execute(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
                
                if (executed) {
                    RCLCPP_INFO(this->get_logger(), "Motion executed successfully! Moved to home position.");
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to execute the motion plan.");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to create a motion plan.");
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveToHomeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}