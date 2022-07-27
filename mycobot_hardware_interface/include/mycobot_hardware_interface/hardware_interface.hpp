#pragma once

#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <memory>
#include <mycobot/mycobot.hpp>
#include <rclcpp/macros.hpp>
#include <string>
#include <vector>

using hardware_interface::return_type;

namespace mycobot {

class MyCobotHardwareInterface : public   hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MyCobotHardwareInterface)

  // Export interfaces
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // livecycle node transitions
  return_type configure(hardware_interface::HardwareInfo const& info) override;
  return_type start(
      ) override;
  return_type stop(
      ) override;

  // hardware read and write
  hardware_interface::return_type read() override;
  hardware_interface::return_type write() override;

 private:
  std::unique_ptr<MyCobot> mycobot_ = nullptr;

  std::array<std::string, 6> joint_names_ = {"joint1", "joint2", "joint3",
                                             "joint4", "joint5", "joint6"};
  std::array<double, 6> arm_position_state_ = {0, 0, 0, 0, 0, 0};
  std::array<double, 6> arm_position_command_ = {0, 0, 0, 0, 0, 0};
};

}  // namespace mycobot
