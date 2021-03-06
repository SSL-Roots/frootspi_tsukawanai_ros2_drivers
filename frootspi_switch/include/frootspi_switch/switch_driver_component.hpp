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

#ifndef FROOTSPI_SWITCH__SWITCH_DRIVER_COMPONENT_HPP_
#define FROOTSPI_SWITCH__SWITCH_DRIVER_COMPONENT_HPP_

#include <memory>
#include <string>
#include <vector>

#include "frootspi_switch/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"

namespace frootspi_switch
{

class Driver : public rclcpp_lifecycle::LifecycleNode
{
public:
  FROOTSPI_SWITCH_PUBLIC
  explicit Driver(const rclcpp::NodeOptions & options);

protected:
  void on_gpio_polling_timer();

private:
  int pi_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>> set_led_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>> switch_state_pub_;

  rclcpp::TimerBase::SharedPtr gpio_polling_timer_;

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

}  // namespace frootspi_switch

#endif  // FROOTSPI_SWITCH__SWITCH_DRIVER_COMPONENT_HPP_