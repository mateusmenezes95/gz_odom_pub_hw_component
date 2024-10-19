// Copyright (c) 2019-2023, SENAI Cimatec
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gtest/gtest.h>

#include <iostream>
#include <memory>
#include <string>

#include "pluginlib/class_loader.hpp"

#include "gz_odom_pub_hw_component/gz_odom_pub_hw_component.hpp"

#include "gz_ros2_control/gz_system_interface.hpp"

TEST(GazeboSystemLoader, Successfully_load_the_hw_component_plugin)
{
  std::unique_ptr<pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>
  pluginlib_loader_ptr_;

  EXPECT_NO_THROW(
    pluginlib_loader_ptr_ = std::make_unique<
      pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface>>(
      "gz_ros2_control",
      "gz_ros2_control::GazeboSimSystemInterface"));
  auto plugin_ = pluginlib_loader_ptr_->createUniqueInstance(
    "gz_odom_pub_hw_component/GzOdomPubHwComponent");
  EXPECT_TRUE(
    pluginlib_loader_ptr_->isClassLoaded(
      "gz_odom_pub_hw_component/GzOdomPubHwComponent"));
}
