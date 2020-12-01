/**
 * @file Viewer.h
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */ 


#pragma once
#ifndef _VIEWER_H
#define _VIEWER_H

#include "ros/ros.h"
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class Viewer
{
public:
  Viewer()
  {
    point_size_ = 2;
    axis_scale_ = 1;
    normals_scale_ = 0.02;
    setColors();
    configViewer();
  }

  ~Viewer(){};

  struct pc_color
  {
    double r, g, b;
  };
  pc_color RED, GREEN, BLUE, PINK, ORANGE, WHITE, PURPLE, YELLOW, BROWN, LIME, CHERRY, SKY, PEACH;
  double point_size_, axis_scale_, normals_scale_;
  bool pressed_space_;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer{ new pcl::visualization::PCLVisualizer("Pointcloud "
                                                                                                      "Viewer") };

  void setColors()
  {
    RED     = { 1, 0, 0 };
    GREEN   = { 0, 1, 0 };
    BLUE    = { 0, 0, 1 };
    PINK    = { 1, 0, 0.5 };
    ORANGE  = { 1, 0.5, 0 };
    WHITE   = { 1, 1, 1 };
    PURPLE  = { 0.4, 0, 1 };
    YELLOW  = { 1, 1, 0.3 };
    BROWN   = { 0.7, 0.4, 0.1};
    LIME    = { 0.9, 1, 0};
    CHERRY  = { 1, 0, 0.5};
    SKY     = { 0.3, 1, 1};
    PEACH   = { 1, 0.8, 0.6};
  }

  void configViewer()
  {
    _viewer->setBackgroundColor(0, 0, 0);
    _viewer->addCoordinateSystem(axis_scale_);
    _viewer->initCameraParameters();
  }

  void loopViewer()
  {
    ROS_WARN("Press (X) on viewer to continue");
    while (!_viewer->wasStopped())
    {
      _viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }

  void closeViewer()
  {
    _viewer->close();
  }

  void resetViewer()
  {
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
  }

  void setPointSize(double point_size)
  {
    point_size_ = point_size;
  }

  void setAxisScale(double axis_scale)
  {
    axis_scale_ = axis_scale;
    _viewer->addCoordinateSystem(axis_scale_);
  }

  void setNormalsScale(double normals_scale)
  {
    normals_scale_ = normals_scale;
  }

  void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* nothing)
  {
    if (event.getKeySym() == "space" && event.keyDown())
      pressed_space_ = true;
  }

  void checkForSpaceKeyPressed()
  {
    _viewer->registerKeyboardCallback(&Viewer::keyboardCallback, *this);
  }

  template <typename PointT>
  void addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, const pc_color& color, const std::string& name)
  {
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_rgb(cloud, color.r, color.g, color.b);
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
      ROS_INFO("Update cloud in Viewer");
      _viewer->updatePointCloud(cloud, cloud_rgb, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }
    else
    {
      ROS_INFO("Add cloud in Viewer");
      _viewer->addPointCloud<PointT>(cloud, cloud_rgb, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }

    loopViewer();
  }

  template <typename PointT>
  void addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, const std::string& name)
  {
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
      ROS_INFO("Update cloud in Viewer");
      _viewer->updatePointCloud(cloud, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }
    else
    {
      ROS_INFO("Add cloud in Viewer");
      _viewer->addPointCloud<PointT>(cloud, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }

    loopViewer();
  }

  template <typename PointT, typename CloudT>
  void addNormalsToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, 
                          typename pcl::PointCloud<CloudT>::Ptr normals,
                          const std::string& name)
  {
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
      ROS_INFO("Update normals in Viewer");
      deletePCFromViewer(name);
      _viewer->addPointCloudNormals<PointT, CloudT>(cloud, normals, 1, normals_scale_, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }
    else
    {
      ROS_INFO("Add normals in Viewer");
      _viewer->addPointCloudNormals<PointT, CloudT>(cloud, normals, 1, normals_scale_, name);
      _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, point_size_, name);
    }
  }                        

  template <typename PointT>
  void addCorrespondencesToViewer(typename pcl::PointCloud<PointT>::Ptr source_cloud,
                                  typename pcl::PointCloud<PointT>::Ptr target_cloud,
                                  pcl::CorrespondencesPtr correspondences)
  {
    _viewer->resetStoppedFlag();

    ROS_INFO("Add correspondences between clouds in Viewer");
    _viewer->addCorrespondences<PointT>(source_cloud, target_cloud, *correspondences);

    loopViewer();
  }                                

  void deletePCFromViewer(const std::string& name)
  {
    _viewer->resetStoppedFlag();

    ROS_INFO("Remove cloud from Viewer");
    _viewer->removePointCloud(name);
  }

  void deleteCorrespondencesFromViewer()
  {
    _viewer->resetStoppedFlag();

    ROS_INFO("Remove correspondences from Viewer");
    _viewer->removeCorrespondences();
  }

  template <typename PointT>
  static void visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer(
        new pcl::visualization::PCLVisualizer("Pointcloud viewer"));
    ROS_INFO("Display cloud in Viewer");

    pc_viewer->setBackgroundColor(0, 0, 0);
    pc_viewer->addPointCloud<PointT>(cloud, "cloud");
    pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    pc_viewer->addCoordinateSystem(1.0);
    pc_viewer->initCameraParameters();

    while (!pc_viewer->wasStopped())
    {
      pc_viewer->spinOnce();
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
    pc_viewer->close();
  }

  static void visualizeMesh(pcl::PolygonMesh::Ptr mesh)
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mesh_viewer(
        new pcl::visualization::PCLVisualizer("Mesh mesh_viewer"));
    ROS_INFO("Display mesh in Viewer");

    mesh_viewer->setBackgroundColor(0, 0, 0);
    mesh_viewer->addPolygonMesh(*mesh, "meshes", 0);
    mesh_viewer->addCoordinateSystem(1.0);
    mesh_viewer->initCameraParameters();

    while (!mesh_viewer->wasStopped())
    {
      mesh_viewer->spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
  }
};

#endif