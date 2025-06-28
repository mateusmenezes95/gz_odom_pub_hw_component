// Copyright (c) 2024 Mateus Menezes

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

#include "gz_odom_pub_hw_component/gz_odom_pub_hw_component.hpp"

#include <fmt/compile.h>

#include <array>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gz_odom_pub_hw_component
{

hardware_interface::CallbackReturn GzOdomPubHwComponent::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  RCLCPP_INFO(this->get_logger(), "Initializing %s hardware component",
    this->get_name().c_str());

  if (hardware_interface::SensorInterface::on_init(hardware_info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto set_hw_param = [&](auto & param_value, const std::string & param_name) {
      if (hardware_info.hardware_parameters.find(param_name) ==
        hardware_info.hardware_parameters.end())
      {
        throw std::runtime_error(
          fmt::format(
            FMT_COMPILE("Required parameter '{}' not found in hardware info"), param_name));
      }

      param_value = hardware_info.hardware_parameters.at(param_name);

      RCLCPP_INFO(this->get_logger(),
        "Parameter '%s' set to '%s'", param_name.c_str(), param_value.c_str());
    };

  try {
    set_hw_param(params_.odom_topic_name, kOdomTopicParamName);
    set_hw_param(params_.ref_frame, kRefFrameParamName);
  } catch(const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Fatal error during initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto complain_about_ifaces_set_in_urdf =
    [&](std::unordered_map<std::string, hardware_interface::InterfaceDescription> & ifaces) {
      if (ifaces.empty()) {
        return;
      }

      std::stringstream ss;
      for (const auto & [name, _] : ifaces) ss << "'" << name << "'";

      RCLCPP_WARN(
        this->get_logger(),
        "The following interfaces were set in the URDF but are not used by this component: %s.",
        ss.str().c_str());

      ifaces.clear();
    };

  // This component export only the states interfaces related to the odometry data,
  // so interfaces set in the URDF will be ignored.
  complain_about_ifaces_set_in_urdf(joint_state_interfaces_);
  complain_about_ifaces_set_in_urdf(sensor_state_interfaces_);

  RCLCPP_INFO(
    this->get_logger(), "%s hardware component initialized successfully",
    this->get_name().c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn GzOdomPubHwComponent::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_INFO(
    this->get_logger(), "Configuring %s component. Previous state: %s",
    this->get_name().c_str(), previous_state.label().c_str());

  if (params_.ref_frame != "enu" && params_.ref_frame != "ned") {
    RCLCPP_WARN(
      this->get_logger(),
      "Invalid reference frame '%s'. Supported frames are 'enu' and 'ned'. Defaulting to 'enu'.",
      params_.ref_frame.c_str());
    params_.ref_frame = "enu";
    enu_to_ned_helper_ = 1.0;  // Default to ENU
  }

  if (params_.ref_frame == "ned") {
    enu_to_ned_helper_ = -1.0;  // Convert ENU to NED
  }

  auto odom_cb = [this](const gz::msgs::Odometry & msg) -> void {
    odom_rt_buffer_.set(msg);
  };

  if (!gz_node_.Subscribe<gz::msgs::Odometry>(params_.odom_topic_name, odom_cb)) {
    RCLCPP_ERROR(
      this->get_logger(), "Failed to subscribe to '%s' topic", params_.odom_topic_name.c_str());
    return hardware_interface::CallbackReturn::FAILURE;
  }

  for (const auto & [name, desc] : sensor_state_interfaces_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Sensor state interface '%s' with data type '%s' configured",
      name.c_str(), desc.get_data_type_string().c_str());
  }

  RCLCPP_INFO(
    this->get_logger(), "%s hardware component configured successfully",
    this->get_name().c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::InterfaceDescription>
GzOdomPubHwComponent::export_unlisted_state_interface_descriptions()
{
  std::vector<hardware_interface::InterfaceDescription> iface_descriptions;
  std::array<std::string, kNumberOfStateInterfaces> state_interface_names = {
    "angular_vel_x", "angular_vel_y", "angular_vel_z",
    "linear_vel_x", "linear_vel_y", "linear_vel_z",
    "quaternion_w", "quaternion_x", "quaternion_y", "quaternion_z",
    "roll", "pitch", "yaw",
    "position_x", "position_y", "position_z"
  };
  hardware_interface::InterfaceInfo iface_info;

  for (const auto & name : state_interface_names) {
    iface_info.name = name;
    iface_info.data_type = "double";  // All interfaces are of type double
    iface_descriptions.emplace_back("odom", iface_info);
  }

  return iface_descriptions;
}

hardware_interface::return_type GzOdomPubHwComponent::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::ignore = time;
  std::ignore = period;

  // This is not real-time safe, but it is not expected to be called in a real-time context
  // as this component should be used in simulation environments where
  // the odometry data is published by a Gazebo plugin.
  odom_msg_ = odom_rt_buffer_.get();

  this->set_state("odom/angular_vel_x", odom_msg_.twist().angular().x());
  this->set_state("odom/angular_vel_y", odom_msg_.twist().angular().y() * enu_to_ned_helper_);
  this->set_state("odom/angular_vel_z", odom_msg_.twist().angular().z() * enu_to_ned_helper_);

  this->set_state("odom/linear_vel_x", odom_msg_.twist().linear().x());
  this->set_state("odom/linear_vel_y", odom_msg_.twist().linear().y() * enu_to_ned_helper_);
  this->set_state("odom/linear_vel_z", odom_msg_.twist().linear().z() * enu_to_ned_helper_);

  this->set_state("odom/quaternion_w", odom_msg_.pose().orientation().w());
  this->set_state("odom/quaternion_x", odom_msg_.pose().orientation().x());
  this->set_state("odom/quaternion_y", odom_msg_.pose().orientation().y());
  this->set_state("odom/quaternion_z", odom_msg_.pose().orientation().z());

  orientation_quaternion_.Set(odom_msg_.pose().orientation().w(),
                              odom_msg_.pose().orientation().x(),
                              odom_msg_.pose().orientation().y(),
                              odom_msg_.pose().orientation().z());

  this->set_state("odom/roll", orientation_quaternion_.Euler().X());
  this->set_state("odom/pitch", orientation_quaternion_.Euler().Y() * enu_to_ned_helper_);
  this->set_state("odom/yaw", orientation_quaternion_.Euler().Z() * enu_to_ned_helper_);

  this->set_state("odom/position_x", odom_msg_.pose().position().x());
  this->set_state("odom/position_y", odom_msg_.pose().position().y() * enu_to_ned_helper_);
  this->set_state("odom/position_z", -odom_msg_.pose().position().z() * enu_to_ned_helper_);

  return hardware_interface::return_type::OK;
}

}  // namespace gz_odom_pub_hw_component

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_odom_pub_hw_component::GzOdomPubHwComponent,
  hardware_interface::SensorInterface
)
