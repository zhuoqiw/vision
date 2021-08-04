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

#include "point_cloud_analyse/point_cloud_analyse.hpp"

#include <algorithm>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "pcl/features/normal_3d.h"
#include "pcl/segmentation/region_growing.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include <pcl/conversions.h>
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLHeader.h>

namespace point_cloud_analyse
{

void toPCL(const sensor_msgs::msg::PointField &pf, pcl::PCLPointField &pcl_pf)
{
  pcl_pf.name = pf.name;
  pcl_pf.offset = pf.offset;
  pcl_pf.datatype = pf.datatype;
  pcl_pf.count = pf.count;
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

void Save(pcl::PointCloud<pcl::Normal>::Ptr normal, const std::string & fileName)
{
  std::ofstream ofile(fileName);
  ofile << std::fixed;
  for (const auto & p : *normal) {
    ofile <<
      p.normal[0] * 1000 << ' ' <<
      p.normal[1] * 1000 << ' ' <<
      p.normal[2] * 1000 << ' ' <<
      p.curvature * 1000 << '\n';
  }
}

void Save(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, const std::string & fileName)
{
  std::ofstream ofile(fileName);
  ofile << std::fixed;
  for (const auto & p : *cloud) {
    ofile <<
      p.x * 1000 << ' ' <<
      p.y * 1000 << ' ' <<
      p.z * 1000 << ' ' <<
      p.intensity * 1000 << '\n';
  }
}

class PointCloudAnalyse::_Impl
{
public:
  explicit _Impl(PointCloudAnalyse * ptr)
  : _node(ptr)
  {
  }

  ~_Impl()
  {
  }

  pcl::ModelCoefficients Plane(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointIndices & inliers)
  {
    pcl::ModelCoefficients coefficients;
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.02);
    seg.setInputCloud(cloud);
    seg.segment(inliers, coefficients);

    return coefficients;
  }

  pcl::ModelCoefficients Plane(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    pcl::PointIndices inliers;
    return Plane(cloud, inliers);
  }

  void Segment(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    std::vector<pcl::PointIndices> & regions)
  {
    // Downsample
    /*pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(0.01, 0.2, 0.01);
    vg.filter(*cloud_grid);*/

    // Estimate normals
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::search::Search<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setKSearch(25);
    ne.compute(*normal);

    pcl::io::savePCDFileASCII("/home/ubuntu/http_server/cloud.pcd", *cloud);

    // Region grow
    pcl::RegionGrowing<pcl::PointXYZI, pcl::Normal> rg;
    rg.setMinClusterSize(1000);
    rg.setMaxClusterSize(1000000);
    rg.setSearchMethod(tree);
    rg.setNumberOfNeighbours(25);
    rg.setInputCloud(cloud);
    rg.setInputNormals(normal);
    rg.setSmoothnessThreshold(5.0 / 180.0 * M_PI);
    rg.setCurvatureThreshold(1.0);
    rg.extract(regions);

    // Sort region
    for (size_t i = 0; i < regions.size(); ++i) {
      auto & ri = regions[i];
      for (size_t j = i + 1; j < regions.size(); ++j) {
        auto & rj = regions[j];
        if (rj.indices.size() > ri.indices.size()) {
          std::swap(ri, rj);
        }
      }
    }

    for (size_t i = 0; i < regions.size(); ++i) {
      for (auto j : regions[i].indices) {
        (*cloud)[j].intensity = i;
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr Extract(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointIndices::Ptr indicePtr)
  {
    pcl::ExtractIndices<pcl::PointXYZI> ei;
    ei.setInputCloud(cloud);
    ei.setIndices(indicePtr);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ret(new pcl::PointCloud<pcl::PointXYZI>);
    ei.filter(*ret);
    return ret;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr Extract(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    pcl::PointIndices & indice)
  {
    pcl::PointIndices::Ptr indicePtr(new pcl::PointIndices(indice));
    return Extract(cloud, indicePtr);
  }

  std::pair<double, double> Statistic(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    if (cloud->width == 0) {return {0, 0};}

    std::vector<double> v;
    v.reserve(cloud->width);

    for (const auto & p : *cloud) {v.push_back(p.intensity);}
    auto sum = std::accumulate(v.begin(), v.end(), 0.0);
    auto mean = sum / v.size();
    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](double x) {return x - mean;});
    auto dev = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.) / v.size();

    return {mean, std::sqrt(dev)};
  }

  std::vector<double> VirtualRuler(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    const int L = 4;
    std::vector<pcl::PointIndices> lines(L);

    const double P[L][4] = {
      {1, 0, 0, 0},
      {0, 0, 1, 0},
      {std::sqrt(0.5), 0, std::sqrt(0.5), 0},
      {std::sqrt(0.5), 0, -std::sqrt(0.5), 0}
    };

    for (uint32_t i = 0; i < cloud->width; ++i) {
      const auto & p = cloud->at(i);
      for (int j = 0; j < L; ++j) {
        auto d = std::abs(p.x * P[j][0] + p.y * P[j][1] + p.z * P[j][2] + P[j][3]);
        if (d < 0.02) {lines[j].indices.push_back(i);}
      }
    }

    auto line0 = Extract(cloud, lines[0]);
    std::pair<double, double> s0 = Statistic(line0);

    auto line1 = Extract(cloud, lines[1]);
    std::pair<double, double> s1 = Statistic(line1);

    auto line2 = Extract(cloud, lines[2]);
    std::pair<double, double> s2 = Statistic(line2);

    auto line3 = Extract(cloud, lines[3]);
    std::pair<double, double> s3 = Statistic(line3);

    return {
/* *INDENT-OFF* */
      s0.first * 1000, s0.second * 1000,
      s1.first * 1000, s1.second * 1000,
      s2.first * 1000, s2.second * 1000,
      s3.first * 1000, s3.second * 1000
/* *INDENT-ON* */
    };
  }

  std::vector<double> CalculatePlaneFlat(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    std::vector<pcl::PointIndices> regions;
    Segment(cloud, regions);
    if (regions.empty()) {
      Save(cloud, "/home/ubuntu/http_server/test_pcd5.txt");
      throw std::runtime_error("CalculatePlaneFlat: no planes");
    }

    // Extract indices
    auto cloud0 = Extract(cloud, regions[0]);

    // Fit plane
    auto c0 = Plane(cloud0);

    for (auto & p : *cloud0) {
      p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
        p,
        c0.values[0],
        c0.values[1],
        c0.values[2],
        c0.values[3]);
    }

    Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");

    /*auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud0, *msg);
    msg->header.stamp = _node->now();
    msg->header.frame_id = _node->_frameID;
    _node->_pub->publish(std::move(msg));*/

    return VirtualRuler(cloud0);
  }

  std::vector<double> CalculatePlaneHV(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    bool vertical)
  {
    std::vector<pcl::PointIndices> regions;
    Segment(cloud, regions);

    if (regions.empty()) {
      Save(cloud, "/home/ubuntu/http_server/test_pcd5.txt");
      throw std::runtime_error("CalculatePlaneFlat: no planes");
    }

    // Extract indices
    auto cloud0 = Extract(cloud, regions[0]);

    // Fit plane
    auto c0 = Plane(cloud0);

    for (auto & p : *cloud0) {
      p.intensity = pcl::pointToPlaneDistanceSigned<pcl::PointXYZI>(
        p,
        c0.values[0],
        c0.values[1],
        c0.values[2],
        c0.values[3]);
    }

    Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");

    Eigen::Affine3f t;
    pcl::getTransformation(0., 0., 0., _node->_roll, _node->_pitch, _node->_yaw, t);
    Eigen::Vector3f v(c0.values[0], c0.values[1], c0.values[2]);
    auto c1 = t * v;

    /*auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud0, *msg);
    msg->header.stamp = _node->now();
    msg->header.frame_id = _node->_frameID;
    _node->_pub->publish(std::move(msg));*/

    auto ret = asin(c1(2)) / M_PI * 180.;

    if (vertical) {
      return {abs(ret)};
    } else {
      return {90. - abs(ret)};
    }
  }

  std::vector<double> CalculateIncludedAngle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
  {
    std::vector<pcl::PointIndices> regions;
    Segment(cloud, regions);

    if (regions.size() < 2) {
      Save(cloud, "/home/ubuntu/http_server/test_pcd5.txt");
      throw std::runtime_error("CalculateIncludedAngle: not enough planes");
    }

    // Extract indices
    auto cloud0 = Extract(cloud, regions[0]);
    auto cloud1 = Extract(cloud, regions[1]);

    // Fit plane
    auto c0 = Plane(cloud0);
    auto c1 = Plane(cloud1);

    *cloud0 += *cloud1;
    Save(cloud0, "/home/ubuntu/http_server/test_pcd5.txt");

    /*auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*cloud0, *msg);
    msg->header.stamp = _node->now();
    msg->header.frame_id = _node->_frameID;
    _node->_pub->publish(std::move(msg));*/

    // Calculate included angle
    auto rd =
      c0.values[0] * c1.values[0] +
      c0.values[1] * c1.values[1] +
      c0.values[2] * c1.values[2];
    return {acos(rd) / M_PI * 180.};
  }

private:
  PointCloudAnalyse * _node;

  std::mutex _mutex;              ///< Mutex to protect shared storage
  std::condition_variable _con;   ///< Conditional variable rely on mutex
  std::deque<sensor_msgs::msg::Image::UniquePtr> _deq;
  std::thread _thread;
};

PointCloudAnalyse::PointCloudAnalyse(const rclcpp::NodeOptions & options)
: Node("point_cloud_analyse_node", options)
{
  _init = std::thread(&PointCloudAnalyse::_Init, this);
}

PointCloudAnalyse::~PointCloudAnalyse()
{
  _init.join();

  _sub.reset();
  _impl.reset();

  RCLCPP_INFO(this->get_logger(), "point_cloud_analyse destroyed successfully");
}

void PointCloudAnalyse::_Init()
{
  try {
    _InitializeParameters();

    _UpdateParameters();

    _pubResult = this->create_publisher<shared_interfaces::msg::Float64Array>(_pubResultName, 1);

    _impl = std::make_unique<_Impl>(this);

    _sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      _subName,
      1,
      std::bind(&PointCloudAnalyse::_Sub, this, std::placeholders::_1)
    );

    _subRpy = this->create_subscription<shared_interfaces::msg::RollPitchYaw>(
      _subRpyName,
      10,
      [this](shared_interfaces::msg::RollPitchYaw::UniquePtr ptr)
      {
        _roll = ptr->roll;
        _pitch = ptr->pitch;
        _yaw = ptr->yaw;
      }
    );

    RCLCPP_INFO(this->get_logger(), "Initialized successfully");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
  }
}

void PointCloudAnalyse::_Sub(sensor_msgs::msg::PointCloud2::UniquePtr ptr)
{
  try {
    _UpdateParameters();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    moveFromROSMsg(*ptr, *cloud);
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    switch (_mode) {
      case 0:
        msg->data = _impl->CalculatePlaneFlat(cloud);
        break;
      case 1:
        msg->data = _impl->CalculateIncludedAngle(cloud);
        break;
      case 2:
        msg->data = _impl->CalculatePlaneHV(cloud, false);
        break;
      case 3:
        msg->data = _impl->CalculatePlaneHV(cloud, true);
        break;
      default:
        throw std::runtime_error("Unsupported operation");
    }

    _pubResult->publish(std::move(msg));
  } catch (const std::exception & e) {
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    _pubResult->publish(std::move(msg));
    RCLCPP_WARN(this->get_logger(), "Exception in subscription: %s", e.what());
  } catch (...) {
    auto msg = std::make_unique<shared_interfaces::msg::Float64Array>();
    _pubResult->publish(std::move(msg));
    RCLCPP_WARN(this->get_logger(), "Exception in subscription: unknown");
  }
}

void PointCloudAnalyse::_InitializeParameters()
{
  declare_parameter("frame_id", _frameID);
  declare_parameter("mode", _mode);
}

void PointCloudAnalyse::_UpdateParameters()
{
  get_parameter("frame_id", _frameID);
  get_parameter("mode", _mode);
}

}  // namespace point_cloud_analyse

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(point_cloud_analyse::PointCloudAnalyse)
