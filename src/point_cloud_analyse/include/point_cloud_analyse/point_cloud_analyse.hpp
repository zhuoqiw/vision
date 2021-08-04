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

#ifndef POINT_CLOUD_ANALYSE__POINT_CLOUD_ANALYSE_HPP_
#define POINT_CLOUD_ANALYSE__POINT_CLOUD_ANALYSE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/float64_array.hpp"
#include "shared_interfaces/msg/roll_pitch_yaw.hpp"

namespace point_cloud_analyse
{

class PointCloudAnalyse : public rclcpp::Node
{
public:
  explicit PointCloudAnalyse(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~PointCloudAnalyse();

private:
  void _Init();
  void _InitializeParameters();
  void _UpdateParameters();
  void _Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr);

private:
  std::string _frameID = "inclinometer";
  int _mode = 0;
  double _roll = 0., _pitch = 0., _yaw = 0.;

  // const char* _pubName = "~/pub";
  // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

  const char * _pubResultName = "~/result";
  rclcpp::Publisher<shared_interfaces::msg::Float64Array>::SharedPtr _pubResult;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subName = "~/points";
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr _sub;

  const char * _subRpyName = "~/rpy";
  rclcpp::Subscription<shared_interfaces::msg::RollPitchYaw>::SharedPtr _subRpy;

  std::thread _init;
};

}  // namespace point_cloud_analyse

#endif  // POINT_CLOUD_ANALYSE__POINT_CLOUD_ANALYSE_HPP_
