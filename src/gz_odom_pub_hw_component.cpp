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

#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gz_odom_pub_hw_component
{

bool GzOdomPubHwComponent::initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & _ecm,
    unsigned int update_rate) {
  nh_ = model_nh;
  std::ignore = joints;
  std::ignore = hardware_info;
  std::ignore = _ecm;
  std::ignore = update_rate;

  RCLCPP_INFO(nh_->get_logger(), "Odometry publisher hardware component initialized");

  return true;
}

hardware_interface::CallbackReturn GzOdomPubHwComponent::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  std::ignore = previous_state;

  RCLCPP_INFO(nh_->get_logger(), "Configuring odometry publisher hardware component");

  auto cb = [this](const gz::msgs::Odometry & msg) -> void {
    odom_rt_buffer_.writeFromNonRT(msg);
  };

  if (!gz_node_.Subscribe<gz::msgs::Odometry>("bluerov2/gz_odometry", cb)) {
    RCLCPP_ERROR(nh_->get_logger(), "Failed to subscribe to odometry topic");
    return hardware_interface::CallbackReturn::FAILURE;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type GzOdomPubHwComponent::read(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::ignore = time;
  std::ignore = period;

  odom_msg_ = *odom_rt_buffer_.readFromRT();

  this->set_state("bluerov2/velocity/angular/x", odom_msg_.twist().angular().x());
  this->set_state("bluerov2/velocity/angular/y", -odom_msg_.twist().angular().y());
  this->set_state("bluerov2/velocity/angular/z", -odom_msg_.twist().angular().z());

  this->set_state("bluerov2/velocity/linear/x", odom_msg_.twist().linear().x());
  this->set_state("bluerov2/velocity/linear/y", -odom_msg_.twist().linear().y());
  this->set_state("bluerov2/velocity/linear/z", -odom_msg_.twist().linear().z());

  orientation_quaternion_.Set(odom_msg_.pose().orientation().w(),
                              odom_msg_.pose().orientation().x(),
                              odom_msg_.pose().orientation().y(),
                              odom_msg_.pose().orientation().z());

  this->set_state("bluerov2/orientation/roll", orientation_quaternion_.Euler().X());
  this->set_state("bluerov2/orientation/pitch", orientation_quaternion_.Euler().Y());
  this->set_state("bluerov2/orientation/yaw", orientation_quaternion_.Euler().Z());

  this->set_state("bluerov2/position/x", odom_msg_.pose().position().x());
  this->set_state("bluerov2/position/y", odom_msg_.pose().position().y());
  this->set_state("bluerov2/position/z", -odom_msg_.pose().position().z());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GzOdomPubHwComponent::write(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  std::ignore = time;
  std::ignore = period;
  return hardware_interface::return_type::OK;
}

}  // namespace gz_odom_pub_hw_component

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_odom_pub_hw_component::GzOdomPubHwComponent,
  gz_ros2_control::GazeboSimSystemInterface
)
