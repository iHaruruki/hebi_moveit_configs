#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

class HebiPointMover : public rclcpp::Node {
public:
  HebiPointMover() : Node("hebi_point_mover") {
  }
  
  void init() {
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "hebi_arm");  
    move_group_->setEndEffectorLink("end_effector_1/output");
  
    subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/clicked_point", 10, std::bind(&HebiPointMover::point_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Ready. Please click a point in Rviz");
  }
  
private:
  void point_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) { RCLCPP_INFO(this->get_logger(), "Received coordinates : x=%f, y=%f, z=%f", msg->point.x, msg->point.y, msg->point.z);
  
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg->point;
  
    tf2::Quaternion q;
    q.setRPY(-1.58, -1.43, -0.16);
    target_pose.orientation.x = q.x();
    target_pose.orientation.y = q.y();
    target_pose.orientation.z = q.z();
    target_pose.orientation.w = q.w();
  
    move_group_->setPoseTarget(target_pose);
    move_group_->move();
  }  
  
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
};
  
int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HebiPointMover>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
