// Loads the hardware interface through ros2_control's ResourceManager and checks the
// topic bridge end to end: commands go out on rover/wheel_cmd, state comes back from
// rover/wheel_state.
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rover_msgs/msg/wheel_cmd.hpp"
#include "rover_msgs/msg/wheel_state.hpp"

namespace
{
const char kUrdf[] = R"(
<robot name="rover">
  <link name="base_link"/>
  <link name="left_wheel"/>
  <link name="right_wheel"/>
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="left_wheel"/><axis xyz="0 1 0"/>
  </joint>
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/><child link="right_wheel"/><axis xyz="0 1 0"/>
  </joint>
  <ros2_control name="rover_system" type="system">
    <hardware>
      <plugin>rover_hardware/RoverSystem</plugin>
    </hardware>
    <joint name="left_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <joint name="right_wheel_joint">
      <command_interface name="velocity"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
</robot>)";

constexpr auto kPeriod = std::chrono::milliseconds(10);
}  // namespace

class RoverSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}
};

TEST_F(RoverSystemTest, BridgesCommandsAndState)
{
  auto node = std::make_shared<rclcpp::Node>("fake_firmware");
  rover_msgs::msg::WheelCmd::SharedPtr last_cmd;
  auto sub = node->create_subscription<rover_msgs::msg::WheelCmd>(
    "rover/wheel_cmd", rclcpp::SensorDataQoS(),
    [&](rover_msgs::msg::WheelCmd::SharedPtr m) {last_cmd = m;});
  auto pub = node->create_publisher<rover_msgs::msg::WheelState>(
    "rover/wheel_state", rclcpp::SensorDataQoS());

  hardware_interface::ResourceManagerParams params;
  params.robot_description = kUrdf;
  params.clock = node->get_clock();
  params.logger = node->get_logger();
  params.executor = nullptr;
  hardware_interface::ResourceManager rm(params, true);

  ASSERT_TRUE(rm.state_interface_exists("left_wheel_joint/velocity"));
  ASSERT_TRUE(rm.command_interface_exists("right_wheel_joint/velocity"));

  rclcpp_lifecycle::State active(
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  rm.set_component_state("rover_system", active);

  auto cmd = rm.claim_command_interface("left_wheel_joint/velocity");
  cmd.set_value(3.5);
  auto cmd_r = rm.claim_command_interface("right_wheel_joint/velocity");
  cmd_r.set_value(-1.25);

  rover_msgs::msg::WheelState state;
  state.left_vel = 2.0f;
  state.right_vel = -2.0f;
  state.left_pos = 10.0f;

  const rclcpp::Time t = node->now();
  const rclcpp::Duration d = rclcpp::Duration::from_seconds(0.01);
  for (int i = 0; i < 100 && !(last_cmd && last_cmd->left > 3.0f); ++i) {
    pub->publish(state);
    rm.read(t, d);
    rm.write(t, d);
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(kPeriod);
  }

  ASSERT_TRUE(last_cmd);
  EXPECT_FLOAT_EQ(last_cmd->left, 3.5f);
  EXPECT_FLOAT_EQ(last_cmd->right, -1.25f);

  auto vel = rm.claim_state_interface("left_wheel_joint/velocity");
  auto pos = rm.claim_state_interface("left_wheel_joint/position");
  for (int i = 0; i < 50 && vel.get_optional().value_or(0.0) != 2.0; ++i) {
    pub->publish(state);
    rclcpp::spin_some(node);
    rm.read(t, d);
    std::this_thread::sleep_for(kPeriod);
  }
  EXPECT_DOUBLE_EQ(vel.get_optional().value_or(-99.0), 2.0);
  EXPECT_DOUBLE_EQ(pos.get_optional().value_or(-99.0), 10.0);
}
