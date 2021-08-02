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

#ifndef MQTT_ROS__MQTT_ROS_HPP_
#define MQTT_ROS__MQTT_ROS_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace mqtt_ros
{

class MqttRos : public rclcpp::Node
{
public:
  explicit MqttRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~MqttRos();
  void Publish(std_msgs::msg::String::UniquePtr & ptr);

private:
  void _Init();
  void _Sub(std_msgs::msg::String::UniquePtr ptr);

private:
  const char * _pubName = "~/request";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/response";
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

  std::thread _init;
};

}  // namespace mqtt_ros

#endif  // MQTT_ROS__MQTT_ROS_HPP_
