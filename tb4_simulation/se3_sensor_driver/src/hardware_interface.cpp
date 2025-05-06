#include "se3_sensor_driver/hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <fcntl.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace se3_sensor_driver
{

static bool set_nonblocking(int sockfd)
{
  int flags = fcntl(sockfd, F_GETFL, 0);
  if (flags == -1)
    return false;
  return (fcntl(sockfd, F_SETFL, flags | O_NONBLOCK) != -1);
}

bool Se3SensorDriver::readLineFromSocket(std::string & line)
{
  char buf[128];
  ssize_t bytes_read = recv(socket_fd_, buf, sizeof(buf) - 1, 0);
  if (bytes_read > 0) {
    buf[bytes_read] = '\0';
    socket_buffer_ += std::string(buf);
  }
  size_t pos = socket_buffer_.find('\n');
  if (pos != std::string::npos) {
    line = socket_buffer_.substr(0, pos);
    socket_buffer_.erase(0, pos + 1);
    return true;
  }
  return false;
}

hardware_interface::CallbackReturn Se3SensorDriver::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SensorInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  sensor_id_ = info.hardware_parameters.at("sensor_id");
  ip_address_  = info.hardware_parameters.at("ip_address");

  for (const auto & sensor : info.sensors)
  {
    sensor_states.emplace_back(sensor.state_interfaces.size(), std::numeric_limits<double>::quiet_NaN());
  }

  socket_fd_ = socket(AF_INET, SOCK_STREAM, 0);
  if (socket_fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("Se3SensorDriver"), "Failed to create socket");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!set_nonblocking(socket_fd_))
  {
    RCLCPP_WARN(rclcpp::get_logger("Se3SensorDriver"), "Failed to set non-blocking mode");
  }

  struct sockaddr_in server_addr;
  memset(&server_addr, 0, sizeof(server_addr));
  server_addr.sin_family = AF_INET;

  int port_ = 12345;
  if (info.hardware_parameters.find("port") != info.hardware_parameters.end())
  {
    port_ = std::stoi(info.hardware_parameters.at("port"));
  }
  server_addr.sin_port = htons(port_);

  if (inet_pton(AF_INET, ip_address_.c_str(), &server_addr.sin_addr) <= 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Se3SensorDriver"), "Invalid IP address: %s", ip_address_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }



bool connected = false;
  int max_attempts = 10;

  for (int attempt = 1; attempt <= max_attempts; ++attempt)
  {
    if (connect(socket_fd_, (struct sockaddr *)&server_addr, sizeof(server_addr)) == 0)
    {
      connected = true;
      break;
    }

    RCLCPP_WARN(rclcpp::get_logger("Se3SensorDriver"),
                "[%s] Attempt %d: Failed to connect to server at %s:%d, retrying...",
                sensor_id_.c_str(), attempt, ip_address_.c_str(), port_);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (!connected)
  {
    RCLCPP_ERROR(rclcpp::get_logger("Se3SensorDriver"),
                 "[%s] Failed to connect to server at %s:%d after %d attempts",
                 sensor_id_.c_str(), ip_address_.c_str(), port_, max_attempts);
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("Se3SensorDriver"), "Connected to sensor server at %s:%d", ip_address_.c_str(), port_);
  socket_buffer_.clear();

  RCLCPP_INFO(rclcpp::get_logger("Se3SensorDriver"), "Se3Driver Initialized");
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Se3SensorDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t sensor_index = 0; sensor_index < info_.sensors.size(); ++sensor_index)
  {
    const auto & sensor = info_.sensors[sensor_index];
    for (size_t i = 0; i < sensor.state_interfaces.size(); ++i)
    {
      interfaces.emplace_back(hardware_interface::StateInterface(
        sensor.name,
        sensor.state_interfaces[i].name,
        &sensor_states[sensor_index][i]
      ));
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("Se3SensorDriver"), "State Interface Exported");
  return interfaces;
}

hardware_interface::return_type Se3SensorDriver::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  // RCLCPP_INFO(rclcpp::get_logger("Se3SensorDriver"), "Reading sensor data from socket");
  
  for (size_t idx = 0; idx < sensor_states.size(); ++idx)
  {
    std::string line;
    if (readLineFromSocket(line))
    {
      std::istringstream iss(line);
      double x, y, z, qx, qy, qz, qw;
      if (iss >> x >> y >> z >> qx >> qy >> qz >> qw)
      {
        auto & state = sensor_states[idx];
        state[0] = x;
        state[1] = y;
        state[2] = z;
        state[3] = qx;
        state[4] = qy;
        state[5] = qz;
        state[6] = qw;
        
        RCLCPP_INFO(
          rclcpp::get_logger("Se3SensorDriver"),
          "[%s] Received: Position: [%.2f %.2f %.2f] Orientation: [%.2f %.2f %.2f %.2f]",
          info_.sensors[idx].name.c_str(), x, y, z, qx, qy, qz, qw);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("Se3SensorDriver"), "Failed to parse sensor data: %s", line.c_str());
      }
    }
    else {
      RCLCPP_DEBUG(rclcpp::get_logger("Se3SensorDriver"), "No new data available for sensor [%s]", info_.sensors[idx].name.c_str());
    }
  }
  return hardware_interface::return_type::OK;
}

Se3SensorDriver::~Se3SensorDriver()
{
  if (socket_fd_ >= 0) {
    close(socket_fd_);
  }
}

}

PLUGINLIB_EXPORT_CLASS(se3_sensor_driver::Se3SensorDriver, hardware_interface::SensorInterface)
