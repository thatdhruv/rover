#include "rover_hardware/rover_system.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace rover_hardware
{

hardware_interface::CallbackReturn RoverSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != kWheels) {
    RCLCPP_ERROR(
      get_logger(), "expected %zu joints (left, right), got %zu", kWheels, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
      joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        get_logger(), "joint '%s' needs exactly one velocity command interface",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  auto it = info_.hardware_parameters.find("state_timeout");
  if (it != info_.hardware_parameters.end()) {
    state_timeout_s_ = std::stod(it->second);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
RoverSystem::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> out;
  for (size_t i = 0; i < kWheels; ++i) {
    out.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_[i]));
    out.push_back(
      std::make_shared<hardware_interface::StateInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_[i]));
  }
  return out;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
RoverSystem::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> out;
  for (size_t i = 0; i < kWheels; ++i) {
    out.push_back(
      std::make_shared<hardware_interface::CommandInterface>(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &cmd_[i]));
  }
  return out;
}

hardware_interface::CallbackReturn RoverSystem::on_activate(const rclcpp_lifecycle::State &)
{
  node_ = std::make_shared<rclcpp::Node>("rover_hardware_io");
  executor_.add_node(node_);
  state_sub_ = node_->create_subscription<rover_msgs::msg::WheelState>(
    "rover/wheel_state", rclcpp::SensorDataQoS(),
    [this](rover_msgs::msg::WheelState::SharedPtr msg) {
      latest_ = msg;
      last_state_time_ = node_->now();
    });
  cmd_pub_ = node_->create_publisher<rover_msgs::msg::WheelCmd>(
    "rover/wheel_cmd", rclcpp::SensorDataQoS());

  last_state_time_ = node_->now();
  cmd_.fill(0.0);
  vel_.fill(0.0);
  pos_.fill(0.0);
  RCLCPP_INFO(get_logger(), "rover hardware interface active");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // One last zero setpoint so the wheels stop even though the heartbeat ends here.
  if (cmd_pub_) {
    rover_msgs::msg::WheelCmd stop;
    cmd_pub_->publish(stop);
  }
  executor_.remove_node(node_);
  state_sub_.reset();
  cmd_pub_.reset();
  node_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  executor_.spin_some();

  if (!latest_) {
    return hardware_interface::return_type::OK;  // nothing received yet; keep zeros
  }
  if ((node_->now() - last_state_time_).seconds() > state_timeout_s_) {
    // Firmware went quiet. Report zero velocity rather than a stale value so the
    // controllers and the EKF do not keep integrating motion that is not happening.
    vel_.fill(0.0);
    return hardware_interface::return_type::OK;
  }
  pos_[0] = latest_->left_pos;
  pos_[1] = latest_->right_pos;
  vel_[0] = latest_->left_vel;
  vel_[1] = latest_->right_vel;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  rover_msgs::msg::WheelCmd msg;
  msg.left = static_cast<float>(cmd_[0]);
  msg.right = static_cast<float>(cmd_[1]);
  cmd_pub_->publish(msg);
  return hardware_interface::return_type::OK;
}

}  // namespace rover_hardware

PLUGINLIB_EXPORT_CLASS(rover_hardware::RoverSystem, hardware_interface::SystemInterface)
