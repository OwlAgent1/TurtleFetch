#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp> 
class CartesianControllerNode : public rclcpp::Node {
public:
  explicit CartesianControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void controlLoop();


  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> vel_solver_;

  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;

  std::string robot_description_param_;
  std::string sensor_base_frame_, tool_frame_, target_frame_, robot_base_frame_, flange_frame_;
  std::vector<std::string> joint_names_;
  double control_rate_, error_gain_;
  std::string velocity_command_topic_;
};