// Copyright (c) 2025 Mateus Menezes

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <gmock/gmock.h>

#include <memory>
#include <string>
#include <vector>

#include "gz/msgs.hh"
#include "gz/transport.hh"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_test_assets/descriptions.hpp"
#include "ros2_control_test_assets/test_hardware_interface_constants.hpp"

#include "gz_odom_pub_hw_component/gz_odom_pub_hw_component.hpp"

struct GzOdomPubHwComponentTestFixture : public ::testing::Test
{
  GzOdomPubHwComponentTestFixture() = default;

  ~GzOdomPubHwComponentTestFixture() override = default;

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    node = std::make_shared<rclcpp::Node>("gz_odom_pub_hw_component_test_node");
    ASSERT_TRUE(node != nullptr) << "Failed to create test node";
    RCLCPP_INFO(node->get_logger(), "Test node created successfully");

    node->get_logger().set_level(rclcpp::Logger::Level::Debug);

    hw_info = hardware_interface::parse_control_resources_from_urdf(
      ros2_control_test_assets::minimal_robot_urdf)[1];
    hw_info.sensors[0].name = "odom";
  }

  void TearDown() override
  {
    // Cleanup code if needed
  }

  rclcpp::Node::SharedPtr node;
  gz_odom_pub_hw_component::GzOdomPubHwComponent hw_component;
  hardware_interface::HardwareInfo hw_info;
};

TEST_F(GzOdomPubHwComponentTestFixture, Initialization_fails_without_required_parameters)
{
  EXPECT_EQ(
    hw_component.init(hw_info, node->get_logger(), node->get_clock()),
    hardware_interface::CallbackReturn::ERROR);

  hw_info.hardware_parameters["odom_topic_name"] = "dummy_odom_topic";

  EXPECT_EQ(
    hw_component.init(hw_info, node->get_logger(), node->get_clock()),
    hardware_interface::CallbackReturn::ERROR);

  hw_info.hardware_parameters["ref_frame_name"] = "dummy_ref_name";

  // Init must return success because the two required parameters were assigned
  EXPECT_EQ(
    hw_component.init(hw_info, node->get_logger(), node->get_clock()),
    hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(GzOdomPubHwComponentTestFixture, Export_right_number_of_state_interfaces)
{
  std::string odom_topic_name = "/dummy_odom_topic";
  gz::transport::Node gz_node;

  hw_info.hardware_parameters["odom_topic_name"] = odom_topic_name;
  hw_info.hardware_parameters["ref_frame_name"] = "dummy_ref_name";

  EXPECT_EQ(
    hw_component.init(hw_info, node->get_logger(), node->get_clock()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto ifaces = hw_component.on_export_state_interfaces();

  ASSERT_FALSE(ifaces.empty());
  EXPECT_THAT(ifaces, ::testing::SizeIs(16));
}

TEST_F(GzOdomPubHwComponentTestFixture, State_interfaces_are_properly_filled)
{
  std::string odom_topic_name = "/dummy_odom_topic";
  gz::transport::Node gz_node;

  hw_info.hardware_parameters["odom_topic_name"] = odom_topic_name;
  hw_info.hardware_parameters["ref_frame_name"] = "enu";

  EXPECT_EQ(
    hw_component.init(hw_info, node->get_logger(), node->get_clock()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto ifaces = hw_component.on_export_state_interfaces();
  ASSERT_FALSE(ifaces.empty());

  EXPECT_EQ(
    hw_component.on_configure(rclcpp_lifecycle::State()),
    hardware_interface::CallbackReturn::SUCCESS);

  auto odom_pub = gz_node.Advertise<gz::msgs::Odometry>(odom_topic_name);
  ASSERT_TRUE(odom_pub);

  gz::msgs::Odometry odom_msg;
  odom_msg.mutable_pose()->mutable_position()->set_x(1.0);
  odom_msg.mutable_pose()->mutable_position()->set_y(2.0);
  odom_msg.mutable_pose()->mutable_position()->set_z(3.0);
  odom_msg.mutable_pose()->mutable_orientation()->set_x(0.0);
  odom_msg.mutable_pose()->mutable_orientation()->set_y(0.0);
  odom_msg.mutable_pose()->mutable_orientation()->set_z(0.0);
  odom_msg.mutable_pose()->mutable_orientation()->set_w(1.0);
  odom_msg.mutable_twist()->mutable_linear()->set_x(0.1);
  odom_msg.mutable_twist()->mutable_linear()->set_y(0.2);
  odom_msg.mutable_twist()->mutable_linear()->set_z(0.3);
  odom_msg.mutable_twist()->mutable_angular()->set_x(0.01);
  odom_msg.mutable_twist()->mutable_angular()->set_y(0.02);
  odom_msg.mutable_twist()->mutable_angular()->set_z(0.03);

  odom_pub.Publish(odom_msg);
  ASSERT_TRUE(odom_pub.HasConnections());

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  rclcpp::Time time = node->get_clock()->now();
  rclcpp::Duration period(0, 100000000);  // 100
  rclcpp::Time start_time = node->get_clock()->now();
  auto return_type = hw_component.read(time, period);
  EXPECT_EQ(return_type, hardware_interface::return_type::OK);

  EXPECT_EQ(hw_component.get_state("odom/angular_vel_x"), 0.01);
  EXPECT_EQ(hw_component.get_state("odom/angular_vel_y"), 0.02);
  EXPECT_EQ(hw_component.get_state("odom/angular_vel_z"), 0.03);
}
