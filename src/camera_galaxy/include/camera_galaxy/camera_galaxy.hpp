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

#ifndef CAMERA_GALAXY__CAMERA_GALAXY_HPP_
#define CAMERA_GALAXY__CAMERA_GALAXY_HPP_

#include <utility>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace camera_galaxy
{

class CameraGalaxy : public rclcpp::Node
{
public:
  explicit CameraGalaxy(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CameraGalaxy();

  void PublishL(sensor_msgs::msg::Image::UniquePtr & ptr)
  {
    _pubL->publish(std::move(ptr));
  }

  void PublishR(sensor_msgs::msg::Image::UniquePtr & ptr)
  {
    _pubR->publish(std::move(ptr));
  }

private:
  void _Init();
  void _Start(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void _Stop(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

private:
  const char * _pubLName = "~/image_l";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubL;

  const char * _pubRName = "~/image_r";
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pubR;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _srvStartName = "~/start";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStart;

  const char * _srvStopName = "~/stop";
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvStop;

  std::thread _init;
};

}  // namespace camera_galaxy

#endif  // CAMERA_GALAXY__CAMERA_GALAXY_HPP_
