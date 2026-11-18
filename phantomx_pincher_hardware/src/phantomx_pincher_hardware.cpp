#include "phantomx_pincher_hardware/phantomx_pincher_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace phantomx_pincher_hardware
{

PhantomxPincherHardware::CallbackReturn PhantomxPincherHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // One slot per joint listed in the robot description
  const size_t n = info_.joints.size();
  hw_positions_.assign(n, 0.0);
  hw_velocities_.assign(n, 0.0);
  hw_efforts_.assign(n, 0.0);
  hw_commands_.assign(n, 0.0);

  // Optional settings from the description file: <param name="port">...</param>
  auto it = info_.hardware_parameters.find("port");
  if (it != info_.hardware_parameters.end()) {
    port_ = it->second;
  }
  it = info_.hardware_parameters.find("baud_rate");
  if (it != info_.hardware_parameters.end()) {
    baud_rate_ = std::stoi(it->second);
  }

  RCLCPP_INFO(
    rclcpp::get_logger("PhantomxPincherHardware"),
    "Initialised with %zu joints, port %s, baud %d", n, port_.c_str(), baud_rate_);
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PhantomxPincherHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhantomxPincherHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

PhantomxPincherHardware::CallbackReturn PhantomxPincherHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  // Placeholder: later this opens the serial port and switches the servos on
  hw_commands_ = hw_positions_;
  return CallbackReturn::SUCCESS;
}

PhantomxPincherHardware::CallbackReturn PhantomxPincherHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Placeholder: later this switches the servos off and closes the port
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type PhantomxPincherHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Placeholder: later this reads the real servo positions
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhantomxPincherHardware::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // Placeholder: later this sends the targets to the servos
  return hardware_interface::return_type::OK;
}

}  // namespace phantomx_pincher_hardware

PLUGINLIB_EXPORT_CLASS(
  phantomx_pincher_hardware::PhantomxPincherHardware, hardware_interface::SystemInterface)
