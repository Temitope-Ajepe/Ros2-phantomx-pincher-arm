#include "phantomx_pincher_hardware/phantomx_pincher_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace phantomx_pincher_hardware
{

// AX-12A: 1024 ticks cover 300 degrees
constexpr double RAD_PER_TICK = (300.0 * M_PI / 180.0) / 1024.0;

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
  auto logger = rclcpp::get_logger("PhantomxPincherHardware");

  if (!driver_.open(port_, baud_rate_)) {
    RCLCPP_ERROR(logger, "Could not open serial port %s", port_.c_str());
    return CallbackReturn::ERROR;
  }
  driver_.wakeUpSerial();
  
    // The ArbotiX board resets when the port opens: give it time to boot
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  driver_.wakeUpSerial();

  // READ-ONLY version: torque is NOT enabled, so the arm can be moved by hand.
  // read() updates one servo per call, so 5 calls read all 5 servos once.
  for (int i = 0; i < 5; ++i) {
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; ++attempt) {
      ok = (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) ==
            hardware_interface::return_type::OK);
    }
    if (!ok) {
      RCLCPP_ERROR(logger, "Could not read the servos (is the 12V power on?)");
      driver_.close();
      return CallbackReturn::ERROR;
    }
  }
  // Start with commands equal to the measured positions
  hw_commands_ = hw_positions_;

  for (uint8_t id = 1; id <= 5; ++id) {
    RCLCPP_INFO(logger, "Servo %d: %d ticks", id, last_ticks_[id]);
  }
    initialised_ = true;
  return CallbackReturn::SUCCESS;
}

PhantomxPincherHardware::CallbackReturn PhantomxPincherHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  driver_.close();
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type PhantomxPincherHardware::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{

  // Read ONE servo per call: a serial round trip takes about 3 ms,
  // the controller cycle is 4 ms, so 5 servos would not fit in one cycle.
  const uint8_t id = next_servo_;
  uint16_t ticks = 0;
  if (!driver_.read_position(id, ticks)) {
    if (!initialised_ || ++failures_ > 50) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PhantomxPincherHardware"), "Failed to read servo %d", id);
      return hardware_interface::return_type::ERROR;
    }
    // Transient failure: keep the previous value and try the next servo
    RCLCPP_WARN(
      rclcpp::get_logger("PhantomxPincherHardware"),
      "Skipped a failed read of servo %d (%d in a row)", id, failures_);
    next_servo_ = (id % 5) + 1;
    return hardware_interface::return_type::OK;
  }
  failures_ = 0;
  last_ticks_[id] = ticks;

  // Convert ticks to the joint value(s) driven by this servo
  for (size_t i = 0; i < servo_ids_.size(); ++i) {
    if (servo_ids_[i] != id) {
      continue;
    }
    if (i < 4) {
      // arm joints: radians relative to the neutral position
      hw_positions_[i] = (ticks - neutral_ticks_[i]) * RAD_PER_TICK;
    } else {
      // finger joints: linear map between closed and open (placeholder calibration)
      const double fraction =
        (ticks - gripper_closed_ticks_) / (gripper_open_ticks_ - gripper_closed_ticks_);
      hw_positions_[i] = fraction * finger_max_opening_;
    }
  }

  // next servo: 1 -> 2 -> 3 -> 4 -> 5 -> 1 ...
  next_servo_ = (id % 5) + 1;

  // once per second (250 cycles), print the raw ticks so we can calibrate the gripper
  if (++cycle_count_ % 250 == 0) {
    RCLCPP_INFO(
      rclcpp::get_logger("PhantomxPincherHardware"),
      "ticks: pan=%d lift=%d elbow=%d wrist=%d gripper=%d",
      last_ticks_[1], last_ticks_[2], last_ticks_[3], last_ticks_[4], last_ticks_[5]);
  }
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
