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

#include "inclinometer/inclinometer.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

extern "C"
{
  #include "attitude_726.h"
}

namespace inclinometer
{

using namespace std::chrono_literals;

const double PI = std::acos(-1);
const double PI_2 = PI / 2.;

double Deg(double rad)
{
  return rad / PI * 180.;
}

double Rad(double deg)
{
  return deg / 180. * PI;
}

Inclinometer::Inclinometer(const rclcpp::NodeOptions & options)
: Node("inclinometer_node", options), tf_broadcaster(this)
{
  _InitializeParameters();

  _UpdateParameters();

  _pub = this->create_publisher<shared_interfaces::msg::RollPitchYaw>(_pubName, 10);

  if (AttUartOpen() < 0) {
    throw std::runtime_error("Can not open inclinometer");
  }

  _thread = std::thread(&Inclinometer::_Worker, this);

  RCLCPP_INFO(this->get_logger(), "inclinometer initialized successfully");
}

Inclinometer::~Inclinometer()
{
  _thread.join();
  AttUartClose();

  RCLCPP_INFO(this->get_logger(), "inclinometer destroyed successfully");
}

void Inclinometer::_InitializeParameters()
{
  this->declare_parameter("hz", _hz);
  this->declare_parameter("frame", _frame);
  this->declare_parameter("child", _child);
}

void Inclinometer::_UpdateParameters()
{
  this->get_parameter("hz", _hz);
  this->get_parameter("frame", _frame);
  this->get_parameter("child", _child);
}

void Inclinometer::_Worker()
{
  while (rclcpp::ok()) {
    float x = 0, y = 0;
    auto status = AttGetParam(&x, &y);
    if (status < 0) {
      RCLCPP_WARN(this->get_logger(), "Error in inclinometer get param [%d]", status);
    } else {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = this->now();
      transformStamped.header.frame_id = _frame;
      transformStamped.child_frame_id = _child;
      transformStamped.transform.translation.x = 0;
      transformStamped.transform.translation.y = 0;
      transformStamped.transform.translation.z = 0;

      tf2::Quaternion q;
      q.setRPY(Rad(x), Rad(-y), 0.);

      transformStamped.transform.rotation.x = q.x();
      transformStamped.transform.rotation.y = q.y();
      transformStamped.transform.rotation.z = q.z();
      transformStamped.transform.rotation.w = q.w();

      tf_broadcaster.sendTransform(transformStamped);
      shared_interfaces::msg::RollPitchYaw rpy;
      rpy.roll = Rad(x), rpy.pitch = Rad(-y), rpy.yaw = 0.;
      _pub->publish(rpy);
    }

    std::this_thread::sleep_for(std::chrono::microseconds(1000000 / _hz));
  }
}

}  // namespace inclinometer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(inclinometer::Inclinometer)
