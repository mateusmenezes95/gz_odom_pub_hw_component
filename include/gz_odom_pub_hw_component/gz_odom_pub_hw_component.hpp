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

#pragma once

#include <map>
#include <string>
#include <thread>

#include "gz/math/Quaternion.hh"
#include "gz/msgs.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/transport.hh"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_buffer.h"

#include "gz_ros2_control/gz_system_interface.hpp"

namespace gz_odom_pub_hw_component
{

class GzOdomPubHwComponent : public gz_ros2_control::GazeboSimSystemInterface
{
public:
  /**
   * @brief Default constructor
   * 
   */
  GzOdomPubHwComponent() = default;

  /**
   * @brief Default destructor
   * 
   */
  virtual ~GzOdomPubHwComponent() = default;

  /**
   * Hardware components have unique ownership to prevent failed or simultaneous access to the
   * hardware. This ensures that each hardware component is accessed exclusively, avoiding potential
   * conflicts and errors.
   */
  GzOdomPubHwComponent(const GzOdomPubHwComponent & other) = delete;

  /**
   * @brief Move constructor
   * 
   * @param other Other instance to be moved
   */
  GzOdomPubHwComponent(GzOdomPubHwComponent && other) = default;

  /** @copydoc */
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & _ecm,
    unsigned int update_rate) override;

  /**
   * @brief Configure the communication with the Odometry Publisher node
   *
   * @param previous_state The previous state of the component
   * @retval hardware_interface::CallbackReturn::FAILURE if the configuration fails
   * @retval hardware_interface::CallbackReturn::SUCCESS if the configuration is successful
   * @retval hardware_interface::CallbackReturn::ERROR if an error occurs during the configuration
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Read the last twist values from the Odometry Publisher Gazebosim plugin
   * 
   * 
   *
   *
   *
   * \param[in] time The time at the start of this control loop iteration
   * \param[in] period The measured time taken by the last control loop iteration
   * \return return_type::OK if the read was successful, return_type::ERROR otherwise.
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Not needed for this component
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
 private:
  /**
   * @brief Gazebo Sim node to communicate with the Gazebo transport system
   *
   * Used to subscribe to the Odometry Publisher plugin
   */
  gz::transport::Node gz_node_;

  /**
   * @brief Local quaternion to store the orientation of the robot
   *
   * Created as member variable to avoid reallocation on each read cycle
   */
  gz::math::Quaternion<double> orientation_quaternion_;

  /**
   * @brief Odometry message to be received from the Odometry Publisher plugin
   */
  gz::msgs::Odometry odom_msg_;

  /**
   * @brief Realtime buffer to store the last received odometry message
   */
  realtime_tools::RealtimeBuffer<gz::msgs::Odometry> odom_rt_buffer_;
};

}  // namespace gz_odom_pub_hw_component
