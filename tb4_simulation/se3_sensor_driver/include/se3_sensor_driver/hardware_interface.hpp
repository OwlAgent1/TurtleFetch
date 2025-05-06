#include "hardware_interface/sensor_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>

namespace se3_sensor_driver
{
class Se3SensorDriver : public hardware_interface::SensorInterface
{
public:
  virtual ~Se3SensorDriver();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool readLineFromSocket(std::string & line);
  int socket_fd_{-1};
  std::string socket_buffer_;
  std::string sensor_id_;
  std::string ip_address_;
  std::vector<std::vector<double>> sensor_states;
};
}
