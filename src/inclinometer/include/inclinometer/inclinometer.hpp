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

#ifndef INCLINOMETER__INCLINOMETER_HPP_
#define INCLINOMETER__INCLINOMETER_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"

namespace inclinometer
{

class Inclinometer : public rclcpp::Node
{
public:
  explicit Inclinometer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~Inclinometer();

private:
  void _InitializeParameters();
  void _UpdateParameters();
  void _Worker();

private:
  int _hz = 10;
  std::string _frame = "map";
  std::string _child = "inclinometer";
  tf2_ros::TransformBroadcaster tf_broadcaster;

  const char * _pubName = "~/rpy";
  rclcpp::Publisher<shared_interfaces::msg::RollPitchYaw>::SharedPtr _pub;

  std::thread _thread;
};

}  // namespace inclinometer

#endif  // INCLINOMETER__INCLINOMETER_HPP_
