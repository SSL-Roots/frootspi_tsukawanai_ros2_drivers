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

#include "frootspi_led/led_driver_component.hpp"

#include <algorithm>
#include <memory>
#include <chrono>
#include <functional>
#include <iostream>
#include <utility>
#include <string>
#include <vector>
#include <pigpiod_if2.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace frootspi_led
{

Driver::Driver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("led_driver", options),
  led_state_(0), pi_(-1)
{
}

void Driver::callback_set_led(const std_msgs::msg::Int16::SharedPtr msg)
{
  led_state_ = msg->data;

  auto pub_msg = std::make_unique<std_msgs::msg::Int16>();
  if(led_state_ == 0 || led_state_ == 1){
    pub_msg->data = led_state_;
    led_state_pub_->publish(std::move(pub_msg));
    gpio_write(pi_, 26, led_state_);
  }
}

CallbackReturn Driver::on_configure(const rclcpp_lifecycle::State &)
{
  using namespace std::placeholders;  // for _1, _2, _3...

  RCLCPP_INFO(this->get_logger(), "on_configure() is called.");

  led_state_pub_ = create_publisher<std_msgs::msg::Int16>("led_state", 1);
  set_led_sub_ = create_subscription<std_msgs::msg::Int16>(
    "set_led", 1, std::bind(&Driver::callback_set_led, this, _1));

  pi_ = pigpio_start(NULL, NULL);

  if(pi_ < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to connect pigpiod.");
    return CallbackReturn::FAILURE;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_activate() is called.");

  led_state_pub_->on_activate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_deactivate() is called.");

  led_state_pub_->on_deactivate();

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_cleanup() is called.");

  led_state_pub_.reset();
  set_led_sub_.reset();

  pigpio_stop(pi_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn Driver::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "on_shutdown() is called.");

  led_state_pub_.reset();
  set_led_sub_.reset();

  return CallbackReturn::SUCCESS;
}

}  // namespace frootspi_led

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(frootspi_led::Driver)
