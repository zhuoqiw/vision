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

#include "mqtt_ros/mqtt_ros.hpp"

#include <memory>
#include <string>
#include <utility>
#include "mqtt/async_client.h"

namespace mqtt_ros
{

class MqttRos::_Impl
{
public:
  explicit _Impl(MqttRos * ptr)
  : _node(ptr), _callback(this)
  {
    _InitializeParameters();

    _UpdateParameters();

    _cli = std::make_shared<mqtt::async_client>(_serverAddress, _clientId, nullptr);
    _cli->set_callback(_callback);
    _cli->connect();
  }

  ~_Impl()
  {
    _cli->disconnect();
  }

  void PublishMqtt(const std::string & str)
  {
    std::size_t found = str.find("status");
    if (found != std::string::npos) {
      _cli->publish(_topic + "/status", str.c_str(), str.size(), _qos, true);
      return;
    }

    found = str.find("result");
    if (found != std::string::npos) {
      _cli->publish(_topic + "/result", str.c_str(), str.size(), _qos, false);
      return;
    }

    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Unrecognized message in PublishMqtt!");
  }

  void PublishRos(const std::string & str)
  {
    auto msg = std::make_unique<std_msgs::msg::String>();
    msg->data = str;
    _node->Publish(msg);
  }

  void Subscribe()
  {
    _cli->subscribe(_topic + "/control", _qos);
  }

private:
  void _InitializeParameters()
  {
    _node->declare_parameter("server_address", _serverAddress);
    _node->declare_parameter("client_id", _clientId);
    _node->declare_parameter("topic", _topic);
    _node->declare_parameter("qos", _qos);
  }

  void _UpdateParameters()
  {
    _node->get_parameter("server_address", _serverAddress);
    _node->get_parameter("client_id", _clientId);
    _node->get_parameter("topic", _topic);
    _node->get_parameter("qos", _qos);
  }

private:
  class Callback : public mqtt::callback
  {
public:
    explicit Callback(_Impl * ptr)
    : _impl(ptr)
    {
    }

private:
    void connected(const std::string & /*cause*/) override
    {
      _impl->Subscribe();
    }

    void connection_lost(const std::string & /*cause*/) override
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "MQTT connection lost!");
      rclcpp::shutdown();
    }

    void message_arrived(mqtt::const_message_ptr msg) override
    {
      _impl->PublishRos(msg->to_string());
    }

    void delivery_complete(mqtt::delivery_token_ptr /*tok*/) override
    {
    }

private:
    _Impl * _impl;
  };

private:
  std::string _serverAddress = "tcp://127.0.0.1:1883";
  std::string _clientId = "paho_cpp_async_subcribe";
  std::string _topic = "iot/zhushi/scanner/1";
  int _qos = 2;

  MqttRos * _node;
  Callback _callback;
  mqtt::async_client::ptr_t _cli;
};

MqttRos::MqttRos(const rclcpp::NodeOptions & options)
: Node("mqtt_ros_node", options)
{
  _init = std::thread(&MqttRos::_Init, this);
}

MqttRos::~MqttRos()
{
  _init.join();

  _sub.reset();
  _impl.reset();
  _pub.reset();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mqtt_ros destroyed successfully");
}

void MqttRos::Publish(std_msgs::msg::String::UniquePtr & ptr)
{
  _pub->publish(std::move(ptr));
}

void MqttRos::_Init()
{
  try {
    _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<std_msgs::msg::String>(
      _subName,
      10,
      std::bind(&MqttRos::_Sub, this, std::placeholders::_1)
    );

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "mqtt_ros initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros initializer: %s", e.what());
    rclcpp::shutdown();
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros initializer: unknown");
    rclcpp::shutdown();
  }
}

void MqttRos::_Sub(std_msgs::msg::String::UniquePtr ptr)
{
  try {
    _impl->PublishMqtt(ptr->data);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros subscription: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Exception in mqtt_ros subscription: unknown");
  }
}

}  // namespace mqtt_ros

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(mqtt_ros::MqttRos)
