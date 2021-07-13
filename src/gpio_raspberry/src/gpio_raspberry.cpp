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

#include "gpio_raspberry/gpio_raspberry.hpp"

#include <fstream>
#include <memory>
#include <string>

namespace gpio_raspberry
{

using std_srvs::srv::Trigger;

void High(int port)
{
  std::ofstream ofile;
  ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  ofile.open(
    std::string("/sys/class/gpio/gpio") +
    std::to_string(port) +
    std::string("/direction")
  );

  ofile << "high" << std::flush;
}

void Low(int port)
{
  std::ofstream ofile;
  ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  ofile.open(
    std::string("/sys/class/gpio/gpio") +
    std::to_string(port) +
    std::string("/direction")
  );

  ofile << "low" << std::flush;
}

void Toggle(int port)
{
  std::ifstream ifile;
  ifile.exceptions(std::ifstream::failbit | std::ifstream::badbit | std::ifstream::eofbit);
  ifile.open(
    std::string("/sys/class/gpio/gpio") +
    std::to_string(port) +
    std::string("/value")
  );

  int flag;
  ifile >> flag;
  ifile.close();

  if (flag) {
    Low(port);
  } else {
    High(port);
  }
}

GpioRaspberry::GpioRaspberry(const rclcpp::NodeOptions & options)
: Node("gpio_raspberry_node", options)
{
  _InitializeParameters();

  _UpdateParameters();

  std::ofstream ofile("/sys/class/gpio/export");
  ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
  ofile << _port << std::flush;

  _srvHigh = this->create_service<Trigger>(
    _srvHighName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Failed: IO set to high";

        High(_port);
        response->success = true;
        response->message = "Success: IO set to high";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service high: %s", e.what());
        rclcpp::shutdown();
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service high: unknown");
        rclcpp::shutdown();
      }
    }
  );

  _srvLow = this->create_service<Trigger>(
    _srvLowName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Failed: IO set to low";

        Low(_port);
        response->success = true;
        response->message = "Success: IO set to low";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service low: %s", e.what());
        rclcpp::shutdown();
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service low: unknown");
        rclcpp::shutdown();
      }
    }
  );

  _srvToggle = this->create_service<Trigger>(
    _srvToggleName,
    [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response)
    {
      try {
        response->success = false;
        response->message = "Failed: IO toggle";

        Toggle(_port);
        response->success = true;
        response->message = "Success: IO toggle";
      } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service toggle: %s", e.what());
        rclcpp::shutdown();
      } catch (...) {
        RCLCPP_ERROR(this->get_logger(), "Exception in service toggle: unknown");
        rclcpp::shutdown();
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

GpioRaspberry::~GpioRaspberry()
{
  try {
    std::ofstream ofile("/sys/class/gpio/unexport");
    ofile.exceptions(std::ofstream::failbit | std::ofstream::badbit);
    ofile << _port << std::flush;

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: unknown");
  }
}

void GpioRaspberry::_InitializeParameters()
{
  this->declare_parameter("port", _port);
}

void GpioRaspberry::_UpdateParameters()
{
  this->get_parameter("port", _port);
}

}  // namespace gpio_raspberry

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gpio_raspberry::GpioRaspberry)
