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

#ifndef LASER_LINE_RECONSTRUCT__LASER_LINE_RECONSTRUCT_HPP_
#define LASER_LINE_RECONSTRUCT__LASER_LINE_RECONSTRUCT_HPP_

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "shared_interfaces/msg/line_center.hpp"

namespace laser_line_reconstruct
{

class LaserLineReconstruct : public rclcpp::Node
{
public:
  explicit LaserLineReconstruct(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~LaserLineReconstruct();

  void Publish(sensor_msgs::msg::PointCloud2::UniquePtr & msg)
  {
    _pub->publish(std::move(msg));
  }

private:
  const char * _pubName = "~/line";
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _pub;

  class _Impl;
  std::unique_ptr<_Impl> _impl;

  const char * _subLName = "~/line_l";
  const char * _subRName = "~/line_r";
  rclcpp::CallbackGroup::SharedPtr _cbgL;
  rclcpp::CallbackGroup::SharedPtr _cbgR;
  rclcpp::Subscription<shared_interfaces::msg::LineCenter>::SharedPtr _subL;
  rclcpp::Subscription<shared_interfaces::msg::LineCenter>::SharedPtr _subR;
};

}  // namespace laser_line_reconstruct

#endif  // LASER_LINE_RECONSTRUCT__LASER_LINE_RECONSTRUCT_HPP_
