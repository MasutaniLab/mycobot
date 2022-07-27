#include "mycobot_hardware_interface/hardware_interface.hpp"

#include <chrono>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <range/v3/all.hpp>
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

namespace views = ::ranges::views;

namespace mycobot {
namespace {
const rclcpp::Logger LOGGER = rclcpp::get_logger("MyCobotHardwareInterface");
}

std::vector<hardware_interface::StateInterface>
MyCobotHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto const& [i, name] : joint_names_ | views::enumerate) {
    interfaces.push_back(hardware_interface::StateInterface(
        name, hardware_interface::HW_IF_POSITION, &arm_position_state_[i]));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
MyCobotHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto const& [i, name] : joint_names_ | views::enumerate) {
    interfaces.push_back(hardware_interface::CommandInterface(
        name, hardware_interface::HW_IF_POSITION, &arm_position_command_[i]));
  }
  return interfaces;
}

return_type MyCobotHardwareInterface::configure(
    const hardware_interface::HardwareInfo& info) {
  if (configure_default(info) !=
      return_type::OK) {
    return return_type::ERROR;
  }

  auto serial_port = mycobot::make_serial_connection_to_robot();

  if (!serial_port) {
    RCLCPP_ERROR(LOGGER, fmt::format("make_serial_connection_to_robot {}",
                                     serial_port.error())
                             .c_str());
    return return_type::ERROR;
  }

  mycobot_ = std::make_unique<mycobot::MyCobot>(std::move(serial_port.value()));

  {
    auto const result = mycobot_->send(power_on());
    if (!result) {
      RCLCPP_ERROR(LOGGER, fmt::format("power_on {}", result.error()));
      return return_type::ERROR;
    }
  }
  {
    auto const result = mycobot_->send(set_color(255, 0, 0));
    if (!result) {
      RCLCPP_ERROR(LOGGER, fmt::format("set_color {}", result.error()));
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  RCLCPP_INFO(LOGGER, "Configured");
  return return_type::OK;
}

return_type MyCobotHardwareInterface::start(
    ) {
  auto const result = mycobot_->send(set_color(0, 255, 0));
  if (!result) {
    RCLCPP_ERROR(LOGGER, fmt::format("set_color {}", result.error()));
    return return_type::ERROR;
  }
  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(LOGGER, "Started");
  return return_type::OK;
}

return_type MyCobotHardwareInterface::stop(
    ) {
  {
    auto const result = mycobot_->send(release_all_servos());
    if (!result) {
      RCLCPP_ERROR(
          LOGGER, fmt::format("release_all_servos {}", result.error()));
      return return_type::ERROR;
    }
  }
  {
    auto const result = mycobot_->send(set_color(0, 0, 255));
    if (!result) {
      RCLCPP_ERROR(LOGGER, fmt::format("set_color {}", result.error()));
      return return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::STOPPED;
  RCLCPP_INFO(LOGGER, "Stopped");
  return return_type::OK;;
}

hardware_interface::return_type MyCobotHardwareInterface::read() {
  auto const positions = mycobot_->get_radians();
  if (!positions) {
    RCLCPP_ERROR(LOGGER,
                 fmt::format("get_radians {}", positions.error()).c_str());
    return hardware_interface::return_type::OK;
  }

  arm_position_state_ = positions.value();
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MyCobotHardwareInterface::write() {
  auto const result = mycobot_->send_radians(arm_position_command_, 100);
  if (!result) {
    RCLCPP_ERROR(LOGGER,
                 fmt::format("send_radians {}", result.error()).c_str());
  }

  return hardware_interface::return_type::OK;
}

}  // namespace mycobot

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mycobot::MyCobotHardwareInterface,
                       hardware_interface::SystemInterface)
