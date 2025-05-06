#include "ros2_controllers_cartesian/cartesian_controller.hpp"
#include <tf2_kdl/tf2_kdl.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

CartesianControllerNode::CartesianControllerNode(const rclcpp::NodeOptions &options)
: Node("cartesian_controller_node", options) {
  declare_parameter<std::string>("robot_description", "");
  declare_parameter<std::string>("sensor_base_frame", "");
  declare_parameter<std::string>("tool_frame", "");
  declare_parameter<std::string>("target_frame", "");
  declare_parameter<std::string>("robot_base_frame", "");
  declare_parameter<std::string>("flange_frame", "");
  declare_parameter<std::vector<std::string>>("joint_names", {});
  declare_parameter<double>("control_rate", 100.0);
  declare_parameter<double>("error_gain", 1.0);
  declare_parameter<std::string>("velocity_command_topic", "/joint_velocities/commands");

  get_parameter("robot_description", robot_description_param_);
  get_parameter("sensor_base_frame", sensor_base_frame_);
  get_parameter("tool_frame", tool_frame_);
  get_parameter("target_frame", target_frame_);
  get_parameter("robot_base_frame", robot_base_frame_);
  get_parameter("flange_frame", flange_frame_);
  get_parameter("joint_names", joint_names_);
  get_parameter("control_rate", control_rate_);
  get_parameter("error_gain", error_gain_);
  get_parameter("velocity_command_topic", velocity_command_topic_);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_description_param_, kdl_tree) ||
      !kdl_tree.getChain(robot_base_frame_, flange_frame_, chain_)) {
    RCLCPP_ERROR(get_logger(), "Failed to construct KDL chain");
    return;
  }
  jac_solver_ = std::make_shared<KDL::ChainJntToJacSolver>(chain_);
  vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);

  joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&CartesianControllerNode::jointStateCallback, this, std::placeholders::_1));

  vel_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(velocity_command_topic_, 10);

  control_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / control_rate_),
    std::bind(&CartesianControllerNode::controlLoop, this));
}

void CartesianControllerNode::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr msg) {
  latest_joint_state_ = msg;
}

void CartesianControllerNode::controlLoop() {
  if (!latest_joint_state_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Waiting for joint state");
    return;
  }
  geometry_msgs::msg::TransformStamped tool_tf, target_tf;
  try {
    tool_tf = tf_buffer_->lookupTransform(robot_base_frame_, tool_frame_, tf2::TimePointZero);
    target_tf = tf_buffer_->lookupTransform(robot_base_frame_, target_frame_, tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    RCLCPP_ERROR(get_logger(), "TF failed: %s", ex.what());
    return;
  }

  KDL::Frame tool_frame_kdl = tf2::transformToKDL(tool_tf);
  KDL::Frame target_frame_kdl = tf2::transformToKDL(target_tf);

  unsigned int nj = joint_names_.size();
  KDL::JntArray q(nj);
  for (unsigned int i = 0; i < nj; ++i) {
    auto it = std::find(
      latest_joint_state_->name.begin(), latest_joint_state_->name.end(), joint_names_[i]);
    if (it == latest_joint_state_->name.end()) {
      RCLCPP_ERROR(get_logger(), "Joint %s not found", joint_names_[i].c_str());
      return;
    }
    size_t idx = std::distance(latest_joint_state_->name.begin(), it);
    q(i) = latest_joint_state_->position[idx];
  }

  KDL::Twist error_twist = KDL::diff(tool_frame_kdl, target_frame_kdl) * error_gain_;

  KDL::Jacobian J(nj);
  jac_solver_->JntToJac(q, J);

  KDL::JntArray qdot(nj);
  vel_solver_->CartToJnt(q, error_twist, qdot);
  RCLCPP_INFO(get_logger(), 
    "qdot=[% .3f, % .3f, % .3f, % .3f, % .3f, % .3f]", qdot(0), qdot(1), qdot(2), qdot(3), qdot(4), qdot(5)
) ;

  // Publish
    std_msgs::msg::Float64MultiArray msg;
    msg.data.resize(nj);
    for (unsigned int i = 0; i < nj; ++i) {
      msg.data[i] = qdot(i);
    }
    vel_pub_->publish(msg);
}