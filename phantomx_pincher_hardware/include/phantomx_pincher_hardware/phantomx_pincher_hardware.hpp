#pragma once

#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace phantomx_pincher_hardware
{

class PhantomxPincherHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  RCLCPP_SHARED_PTR_DEFINITIONS(PhantomxPincherHardware)

  // Called once when the plugin is loaded: read the joint list, set up storage
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  // Tell ros2_control which values we report (state) and accept (command)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Start: open the serial port and switch the servos on
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  // Stop: switch the servos off and close the port
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Called about 25 times per second: read servo positions, then send new targets
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string port_ = "/dev/ttyUSB0";
  int baud_rate_ = 57600;

  // One value per joint, in radians (the controller's language)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
};

}  // namespace phantomx_pincher_hardware  
