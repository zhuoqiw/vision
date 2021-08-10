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

#ifndef MOTOR_ENCODER__MOTOR_ENCODER_HPP_
#define MOTOR_ENCODER__MOTOR_ENCODER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motor_tmcl
{
class MotorTmcl;
}  // namespace motor_tmcl

namespace motor_encoder
{

class MotorEncoder : public rclcpp::Node
{
public:
  explicit MotorEncoder(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~MotorEncoder();

private:
  void _InitializeParameters();
  void _UpdateParameters();

private:
  int _speed = 3200;

  std::unique_ptr<motor_tmcl::MotorTmcl> _motorTmcl;

  const char * _srvScanName = "~/scan";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvScan;

  const char * _srvZeroName = "~/zero";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvZero;

  const char * _srvCenterName = "~/center";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvCenter;
};

}  // namespace motor_encoder

#endif  // MOTOR_ENCODER__MOTOR_ENCODER_HPP_
