// Copyright 2021 Roots
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

#ifndef FROOTSPI_LED__LED_DRIVER_COMPONENT_HPP_
#define FROOTSPI_LED__LED_DRIVER_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "frootspi_led/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"

namespace frootspi_led
{

class Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_LED_PUBLIC
  explicit Driver(const rclcpp::NodeOptions & options);

private:
  int led_state_;
  int pi_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>> led_state_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr set_led_sub_;

  void callback_set_led(const std_msgs::msg::Int16::SharedPtr msg);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);
};

}  // namespace frootspi_led

#endif  // FROOTSPI_LED__LED_DRIVER_COMPONENT_HPP_