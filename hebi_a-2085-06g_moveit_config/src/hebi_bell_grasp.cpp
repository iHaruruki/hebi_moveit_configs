#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/empty.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class HebiArmMover : public rclcpp::Node
{
public:
    HebiArmMover(const rclcpp::NodeOptions &options)
        : Node("hebi_bell_grasp", options)
    {
        callback_group_ =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        trigger_sub_ = this->create_subscription<std_msgs::msg::Empty>(
            "/exec_grasp", 10, std::bind(&HebiArmMover::execute_move_callback, this, std::placeholders::_1), sub_options);

        finish_pub_ = this->create_publisher<std_msgs::msg::Empty>(
            "/move_done", 10);
                
        saved_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/saved_pose", 10);

        RCLCPP_INFO(
            this->get_logger(),
            "Hebi Grasp Node started.");
    }

private:
    constexpr static double lift_height = 0.08;
    constexpr static double j6_offset = 0.00;   // radians
    void execute_move_callback(const std_msgs::msg::Empty::SharedPtr)
    {
        auto move_group =
            moveit::planning_interface::MoveGroupInterface(
                shared_from_this(),
                "hebi_arm");
        move_group.setMaxVelocityScalingFactor(1.0);      // 0.0 - 1.0
        move_group.setMaxAccelerationScalingFactor(0.8);  // 0.0 - 1.0

        // Get current end-effector pose
        geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
        geometry_msgs::msg::PoseStamped lifted_pose = move_group.getCurrentPose();

        // Lift 5 cm
        geometry_msgs::msg::Pose target_pose = current_pose;
        target_pose.position.z += lift_height;

        std::vector<double> current_joint_values = move_group.getCurrentJointValues();
        double current_j6 = current_joint_values[5];

        moveit::core::RobotStatePtr state = std::make_shared<moveit::core::RobotState>(move_group.getRobotModel());

        const moveit::core::JointModelGroup* joint_group = state->getJointModelGroup("hebi_arm");
        state->setJointGroupPositions(joint_group, current_joint_values);
        state->update();

        // Solve IK
        bool found_ik = state->setFromIK(joint_group, target_pose, 0.1);

        if (!found_ik)
        {
            RCLCPP_ERROR(this->get_logger(), "IK failed.");
            return;
        }

        // Copy IK solution
        std::vector<double> target_joint_values;
        state->copyJointGroupPositions(joint_group, target_joint_values);

        // Lock J6 (with optional offset)
        target_joint_values[5] = current_j6 + j6_offset;

        // Plan
        move_group.setJointValueTarget(target_joint_values);

        moveit::planning_interface::MoveGroupInterface::Plan plan;

        bool success =
            (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            
        if (!success)
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed");
            return;
        }

        auto result = move_group.execute(plan);

        if (result == moveit::core::MoveItErrorCode::SUCCESS)
        {
            saved_pose_pub_->publish(lifted_pose);
            RCLCPP_INFO(this->get_logger(), "Published saved pose.");
            finish_pub_->publish(std_msgs::msg::Empty());
        }
    }
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr trigger_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr saved_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr finish_pub_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<HebiArmMover>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
