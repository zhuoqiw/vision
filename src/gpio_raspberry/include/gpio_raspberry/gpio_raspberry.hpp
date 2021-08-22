// Copyright 2019 Zhushi Tech, Inc.
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

#ifndef GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_
#define GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace gpio_raspberry
{

class GpioRaspberry : public rclcpp::Node
{
public:
  explicit GpioRaspberry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~GpioRaspberry();

private:
  void _InitializeParameters();
  void _UpdateParameters();

private:
  int _port = 26;

  const char * _srvHighName = "~/high";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvHigh;

  const char* _srvLowName = "~/low";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvLow;

  const char * _srvToggleName = "~/toggle";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvToggle;
}; 

}  // namespace gpio_raspberry

#endif  // GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_

