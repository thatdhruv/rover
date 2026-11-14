#ifndef ROVER_HARDWARE__ROVER_SYSTEM_HPP_
#define ROVER_HARDWARE__ROVER_SYSTEM_HPP_

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rover_msgs/msg/wheel_cmd.hpp"
#include "rover_msgs/msg/wheel_state.hpp"

namespace rover_hardware
{

/// ros2_control SystemInterface for the rover.
///
/// It talks to the firmware through the micro-ROS agent: wheel setpoints go out on
/// `rover/wheel_cmd`, measurements come back on `rover/wheel_state`. Every write()
/// doubles as the link heartbeat the firmware's watchdog logic expects.
class RoverSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces()
  override;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces()
  override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  static constexpr size_t kWheels = 2;

  std::vector<std::string> joint_names_;
  std::array<double, kWheels> pos_{};
  std::array<double, kWheels> vel_{};
  std::array<double, kWheels> cmd_{};

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  rclcpp::Subscription<rover_msgs::msg::WheelState>::SharedPtr state_sub_;
  rclcpp::Publisher<rover_msgs::msg::WheelCmd>::SharedPtr cmd_pub_;
  rover_msgs::msg::WheelState::SharedPtr latest_;
  rclcpp::Time last_state_time_;
  double state_timeout_s_{0.5};
};

}  // namespace rover_hardware

#endif  // ROVER_HARDWARE__ROVER_SYSTEM_HPP_
