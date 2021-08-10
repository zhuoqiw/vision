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

#ifndef MOTOR__MOTOR_HPP_
#define MOTOR__MOTOR_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motor
{

class Motor : public rclcpp::Node
{
public:
  explicit Motor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Motor();

private:
  void _InitializeParameters();
  void _UpdateParameters();

private:
  int _speed = 3200;

  const char * _srvScanName = "~/scan";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvScan;

  const char * _srvZeroName = "~/zero";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvZero;

  const char * _srvCenterName = "~/center";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvCenter;
};

}  // namespace motor

#endif  // MOTOR__MOTOR_HPP_
