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

#include "motor_encoder/motor_encoder.hpp"

#include <memory>

#include "motor_tmcl.hpp"

namespace motor_encoder
{

using std_srvs::srv::Trigger;

MotorEncoder::MotorEncoder(const rclcpp::NodeOptions & options)
: Node("motor_encoder_node", options)
{
  _InitializeParameters();

  _motorTmcl = std::make_unique<motor_tmcl::MotorTmcl>(nullptr);

  _srvScan = this->create_service<Trigger>(
    _srvScanName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Fail: motor_encoder scan";

        _UpdateParameters();

        _motorTmcl->MotorCtlScan(_speed);

        response->success = true;
        response->message = "Success: motor_encoder scan";
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception in service scan: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Exception in service scan: unknown");
      }
    }
  );

  _srvZero = this->create_service<Trigger>(
    _srvZeroName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Fail: motor_encoder zero";

        _motorTmcl->MotorCtlZero();

        response->success = true;
        response->message = "Success: motor_encoder zero";
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception in service zero: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Exception in service zero: unknown");
      }
    }
  );

  _srvCenter = this->create_service<Trigger>(
    _srvCenterName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Fail: motor_encoder center";

        _motorTmcl->MotorCtlCenter();

        response->success = true;
        response->message = "Success: motor_encoder center";
      } catch (const std::exception & e) {
        RCLCPP_WARN(this->get_logger(), "Exception in service center: %s", e.what());
      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "Exception in service center: unknown");
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "motor_encoder initialized successfully");
}

MotorEncoder::~MotorEncoder()
{
  _srvCenter.reset();
  _srvZero.reset();
  _srvScan.reset();

  _motorTmcl.reset();

  RCLCPP_INFO(this->get_logger(), "motor_encoder destroyed successfully");
}

void MotorEncoder::_InitializeParameters()
{
  this->declare_parameter("speed", _speed);
}

void MotorEncoder::_UpdateParameters()
{
  this->get_parameter("speed", _speed);
}

}  // namespace motor_encoder

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(motor_encoder::MotorEncoder)
