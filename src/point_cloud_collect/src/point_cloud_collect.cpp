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

#include "point_cloud_collect/point_cloud_collect.hpp"

#include <deque>
#include <memory>
#include <string>
#include <utility>

#include "pcl_conversions/pcl_conversions.h"

namespace point_cloud_collect
{

class PointCloudCollect::_Impl
{
public:
  explicit _Impl(PointCloudCollect * ptr)
  : _node(ptr)
  {
    _InitializeParameters();

    _UpdateParameters();

    _InitialCloud();

    _thread = std::thread(&_Impl::_Worker, this);
  }

  ~_Impl()
  {
    _con.notify_all();
    _thread.join();
  }

  void PushBack(sensor_msgs::msg::PointCloud2::UniquePtr & ptr)
  {
    std::unique_lock<std::mutex> lk(_mutex);
    _deq.emplace_back(std::move(ptr));
    lk.unlock();
    _con.notify_all();
  }

private:
  void _Worker()
  {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lk(_mutex);
      if (_deq.empty() == false) {
        auto ptr = std::move(_deq.front());
        _deq.pop_front();
        lk.unlock();
        if (ptr->header.frame_id == "-1") {
          _Publish();
        } else {
          _Collect(ptr);
        }
      } else {
        _con.wait(lk);
      }
    }
  }

  void _Collect(sensor_msgs::msg::PointCloud2::UniquePtr & ptr)
  {
    if (ptr->width != 0) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      pcl::moveFromROSMsg(*ptr, cloud);
      *_cloud += cloud;
    }
  }

  void _Publish()
  {
    auto ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();

    if (_cloud->width != 0) {
      for (auto & p : *_cloud) {p.intensity = 0;}
      pcl::toROSMsg(*_cloud, *ptr);
      ptr->header.stamp = _node->now();
      ptr->header.frame_id = _frameID;
      _node->_pub->publish(std::move(ptr));
    } else {
      RCLCPP_WARN(_node->get_logger(), "Collect nothing");
    }

    _InitialCloud();
  }

  void _InitialCloud()
  {
    _cloud.reset(new pcl::PointCloud<pcl::PointXYZI>);
    _cloud->height = 1;
    _cloud->width = 0;
  }

  void _InitializeParameters()
  {
    _node->declare_parameter("frame_id");
  }

  void _UpdateParameters()
  {
    _node->get_parameter("frame_id", _frameID);
  }

private:
  std::string _frameID = "inclinometer";

  PointCloudCollect * _node;
  pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud;
  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<sensor_msgs::msg::PointCloud2::UniquePtr> _deq;
  std::thread _thread;
};

PointCloudCollect::PointCloudCollect(const rclcpp::NodeOptions & options)
: Node("point_cloud_collect_node", options)
{
  _pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(_pubName, 1);

  _impl = std::make_unique<_Impl>(this);

  _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    _subName,
    50,
    std::bind(&PointCloudCollect::_Sub, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

PointCloudCollect::~PointCloudCollect()
{
  try {
    _sub.reset();
    _impl.reset();
    _pub.reset();

    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in destructor: unknown");
  }
}

void PointCloudCollect::_Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr)
{
  try {
    _impl->PushBack(ptr);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in subscription: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in subscription: unknown");
  }
}

}  // namespace point_cloud_collect

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_collect::PointCloudCollect)
