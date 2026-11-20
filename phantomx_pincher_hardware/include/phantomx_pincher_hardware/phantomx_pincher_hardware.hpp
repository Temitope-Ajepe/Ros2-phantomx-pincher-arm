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
#include "phantomx_pincher_hardware/arbotix_driver.hpp"

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
  
  // Serial driver (from the professor's CPP_test, see README credits)
  ArbotixDriver driver_;

  // One entry per joint, in the order of the robot description:
  // pan, shoulder lift, elbow, wrist, finger1, finger2
  std::vector<uint8_t> servo_ids_ = {1, 2, 3, 4, 5, 5};
  std::vector<int> neutral_ticks_ = {506, 510, 505, 512, 512, 512};

  // Last raw reading of each servo, indexed by servo ID (index 0 unused)
  std::vector<uint16_t> last_ticks_ = std::vector<uint16_t>(6, 0);
  uint8_t next_servo_ = 1;   // we read one servo per cycle (serial is slow)
  unsigned long cycle_count_ = 0;
  bool initialised_ = false;   // true once the first reads of on_activate succeeded
  int failures_ = 0;           // consecutive failed reads

  // Gripper: PLACEHOLDER values, to be calibrated on the real arm
  double gripper_closed_ticks_ = 0.0;
  double gripper_open_ticks_ = 416.0;
  double finger_max_opening_ = 0.0158;
  
 // Safety clamp on the targets sent to the arm joints (radians, from the zero pose).
  // Measured by moving the real arm by hand to its stops, minus ~10 degrees of margin.
  std::vector<double> lower_limit_ = {-1.35, -1.45, -2.25, -1.70};
  std::vector<double> upper_limit_ = { 1.35,  1.43,  2.25,  1.38};

  // Moving speed written to each servo (AX-12A: 0 means MAXIMUM speed!)
  uint16_t moving_speed_ = 60;
  bool torque_on_ = false;   // true only after the safe start-up in on_activate

  // One value per joint, in radians (the controller's language)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> hw_commands_;
};

}  // namespace phantomx_pincher_hardware

