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

#include <pcl/conversions.h>
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLHeader.h>

namespace point_cloud_collect
{

void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
{
  pcl_pf.name = pf.name;
  pcl_pf.offset = pf.offset;
  pcl_pf.datatype = pf.datatype;
  pcl_pf.count = pf.count;
}

void toPCL(const std::vector<sensor_msgs::msg::PointField> &pfs, std::vector<pcl::PCLPointField> &pcl_pfs)
{
  pcl_pfs.resize(pfs.size());
  std::vector<sensor_msgs::msg::PointField>::const_iterator it = pfs.begin();
  int i = 0;
  for(; it != pfs.end(); ++it, ++i) {
    toPCL(*(it), pcl_pfs[i]);
  }
}

void toPCL(const rclcpp::Time &stamp, std::uint64_t &pcl_stamp)
{
  pcl_stamp = stamp.nanoseconds() / 1000ull;  // Convert from ns to us
}

void toPCL(const std_msgs::msg::Header &header, pcl::PCLHeader &pcl_header)
{
  toPCL(header.stamp, pcl_header.stamp);
  // TODO(clalancette): Seq doesn't exist in the ROS2 header
  // anymore.  wjwwood suggests that we might be able to get this
  // information from the middleware in the future, but for now we
  // just set it to 0.
  pcl_header.seq = 0;
  pcl_header.frame_id = header.frame_id;
}

void copyPointCloud2MetaData(const sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
  toPCL(pc2.header, pcl_pc2.header);
  pcl_pc2.height = pc2.height;
  pcl_pc2.width = pc2.width;
  toPCL(pc2.fields, pcl_pc2.fields);
  pcl_pc2.is_bigendian = pc2.is_bigendian;
  pcl_pc2.point_step = pc2.point_step;
  pcl_pc2.row_step = pc2.row_step;
  pcl_pc2.is_dense = pc2.is_dense;
}

void moveToPCL(sensor_msgs::msg::PointCloud2 &pc2, pcl::PCLPointCloud2 &pcl_pc2)
{
  copyPointCloud2MetaData(pc2, pcl_pc2);
  pcl_pc2.data.swap(pc2.data);
}

template<typename T>
void moveFromROSMsg(sensor_msgs::msg::PointCloud2 &cloud, pcl::PointCloud<T> &pcl_cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  moveToPCL(cloud, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, pcl_cloud);
}

rclcpp::Time fromPCL(const std::uint64_t &pcl_stamp)
{
  rclcpp::Time stamp;
  fromPCL(pcl_stamp, stamp);
  return stamp;
}

void fromPCL(const pcl::PCLHeader &pcl_header, std_msgs::msg::Header &header)
{
  header.stamp = fromPCL(pcl_header.stamp);
  header.frame_id = pcl_header.frame_id;
}

void fromPCL(const pcl::PCLPointField &pcl_pf, sensor_msgs::msg::PointField &pf)
{
  pf.name = pcl_pf.name;
  pf.offset = pcl_pf.offset;
  pf.datatype = pcl_pf.datatype;
  pf.count = pcl_pf.count;
}

void fromPCL(const std::vector<pcl::PCLPointField> &pcl_pfs, std::vector<sensor_msgs::msg::PointField> &pfs)
{
  pfs.resize(pcl_pfs.size());
  std::vector<pcl::PCLPointField>::const_iterator it = pcl_pfs.begin();
  int i = 0;
  for(; it != pcl_pfs.end(); ++it, ++i) {
    fromPCL(*(it), pfs[i]);
  }
}

void copyPCLPointCloud2MetaData(const pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::msg::PointCloud2 &pc2)
{
  fromPCL(pcl_pc2.header, pc2.header);
  pc2.height = pcl_pc2.height;
  pc2.width = pcl_pc2.width;
  fromPCL(pcl_pc2.fields, pc2.fields);
  pc2.is_bigendian = pcl_pc2.is_bigendian;
  pc2.point_step = pcl_pc2.point_step;
  pc2.row_step = pcl_pc2.row_step;
  pc2.is_dense = pcl_pc2.is_dense;
}

void moveFromPCL(pcl::PCLPointCloud2 &pcl_pc2, sensor_msgs::msg::PointCloud2 &pc2)
{
  copyPCLPointCloud2MetaData(pcl_pc2, pc2);
  pc2.data.swap(pcl_pc2.data);
}

template<typename T>
void toROSMsg(const pcl::PointCloud<T> &pcl_cloud, sensor_msgs::msg::PointCloud2 &cloud)
{
  pcl::PCLPointCloud2 pcl_pc2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_pc2);
  moveFromPCL(pcl_pc2, cloud);
}

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
      moveFromROSMsg(*ptr, cloud);
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
    _node->declare_parameter("frame_id", _frameID);
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
