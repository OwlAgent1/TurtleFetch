#include "rclcpp/rclcpp.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sstream>

class SensorTfSender : public rclcpp::Node
{
public:
  SensorTfSender()
  : Node("sensor_tf_sender"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    this->declare_parameter<std::string>("tool_frame", "tool");
    this->declare_parameter<std::string>("target_frame", "target");
    this->declare_parameter<std::string>("sensor_frame", "sensor_base");
    this->declare_parameter<int>("port", 12345);
    
    tool_frame_ = this->get_parameter("tool_frame").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    sensor_frame_ = this->get_parameter("sensor_frame").as_string();
    port_ = this->get_parameter("port").as_int();

    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to create server socket");
      rclcpp::shutdown();
      return;
    }
    
    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = INADDR_ANY;
    server_addr.sin_port = htons(port_);
    if (bind(server_fd_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Bind failed on port %d", port_);
      rclcpp::shutdown();
      return;
    }
    
    if (listen(server_fd_, 5) < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Listen failed");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Sensor TF Sender listening on port %d", port_);
    
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    client_fd_ = accept(server_fd_, (struct sockaddr *)&client_addr, &addr_len);
    if (client_fd_ < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to accept client connection");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Client connected to Sensor TF Sender");

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     std::bind(&SensorTfSender::timer_callback, this));
  }

  ~SensorTfSender()
  {
    if (client_fd_ >= 0) { close(client_fd_); }
    if (server_fd_ >= 0) { close(server_fd_); }
  }

private:
void timer_callback()
{
  geometry_msgs::msg::TransformStamped tf;
  try
  {
    tf = tf_buffer_.lookupTransform(sensor_frame_, tool_frame_, tf2::TimePointZero);
  }
  catch (tf2::TransformException & ex)
  {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return;
  }

  std::stringstream ss;
  ss << tf.transform.translation.x << " "
     << tf.transform.translation.y << " "
     << tf.transform.translation.z << " "
     << tf.transform.rotation.x << " "
     << tf.transform.rotation.y << " "
     << tf.transform.rotation.z << " "
     << tf.transform.rotation.w << "\n";

  std::string msg = ss.str();
  ssize_t bytes_sent = send(client_fd_, msg.c_str(), msg.size(), 0);
  if (bytes_sent < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send sensor data");
  }
  else
  {
    RCLCPP_DEBUG(this->get_logger(), "[%s] Sent: %s", tool_frame_.c_str(), msg.c_str());
  }
}


  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  
  std::string tool_frame_, target_frame_, sensor_frame_;
  int port_;
  int server_fd_{-1}, client_fd_{-1};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorTfSender>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
