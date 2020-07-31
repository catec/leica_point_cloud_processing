/**
 * @file Utils.h
 * @author Ines Lara (imlara@catec.aero)
 * @brief useful functions for point cloud processing
 * @version 0.1
 * @date 2020-07-14
 *
 * @copyright Copyright (c) 2020
 *
 */

#pragma once
#ifndef _UTILS_H
#define _UTILS_H

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#endif

class Utils
{
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

private:
  /**
   * @brief This class is not meant to be instantiated.
   *
   */
  Utils(){};

  /**
   * @brief This class is not meant to be instantiated.
   *
   */
  ~Utils(){};

public:
  /**
   * @brief If set, point to specified pointcloud folder.
   *
   */
  static std::string _pc_path;

  /**
   * @brief frame_id to add in PointCloud2. Default to "/world".
   *
   */
  static std::string _frame_id;

  /**
   * @brief Set the pointcloud folder path object.
   *
   * @param pointcloud_folder_path
   */
  static void setPCpath(const std::string& pointcloud_folder_path);

  /**
   * @brief Get the normals for input cloud with specified normal's radius.
   *
   * @param[in] cloud
   * @param[in] normal_radius
   * @param[out] normals
   * @return true
   * @return false
   */
  static bool getNormals(PointCloudRGB::Ptr& cloud, double normal_radius, pcl::PointCloud<pcl::Normal>::Ptr& normals);

  /**
   * @brief Check whether cloud contains data and is not empty.
   *
   * @param cloud
   * @return true
   * @return false
   */
  static bool isValidCloud(PointCloudXYZ::Ptr cloud);

  /**
   * @brief Check whether cloud contains data and is not empty.
   *
   * @param cloud
   * @return true
   * @return false
   */
  static bool isValidCloud(PointCloudRGB::Ptr cloud);

  /**
   * @brief Check whether PointCloud2 contains data and is not empty.
   *
   * @param cloud_msg
   * @return true
   * @return false
   */
  static bool isValidCloudMsg(const sensor_msgs::PointCloud2& cloud_msg);

  /**
   * @brief Apply RGB values to cloud.
   *
   * @param[in-out] cloud_rgb
   * @param R 0~255
   * @param G 0~255
   * @param B 0~255
   */
  static void colorizeCloud(PointCloudRGB::Ptr cloud_rgb, int R, int G, int B);

  /**
   * @brief Convert XYZ cloud to XYZRGB cloud with specified RGB values.
   *
   * @param[in] cloud
   * @param R 0~255
   * @param G 0~255
   * @param B 0~255
   * @param[out] cloud_rgb
   */
  static void cloudToXYZRGB(PointCloudXYZ::Ptr cloud, PointCloudRGB::Ptr cloud_rgb, int R, int G, int B);

  /**
   * @brief Convert XYZRGB cloud to PointCloud2.
   *
   * @param[in] cloud
   * @param[out] cloud_msg
   */
  static void cloudToROSMsg(PointCloudRGB::Ptr cloud, sensor_msgs::PointCloud2& cloud_msg);

  /**
   * @brief Obtain the resolution of the cloud.
   *
   * @param cloud
   * @return double
   */
  static double computeCloudResolution(PointCloudXYZ::Ptr cloud);

  /**
   * @brief Obtain the resolution of the cloud.
   *
   * @param cloud
   * @return double
   */
  static double computeCloudResolution(PointCloudRGB::Ptr cloud);

  /**
   * @brief Print on console transform values with matrix format.
   *
   * @param transform
   */
  static void printTransform(const Eigen::Matrix4f& transform);

  /**
   * @brief Apply extract indices filter to input cloud with given indices.
   *
   * @param[in] cloud_in
   * @param[in] indices
   * @param[out] cloud_out
   */
  static void indicesFilter(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out,
                            boost::shared_ptr<std::vector<int> > indices);

  static void displaceCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, double x_offset = 0,
                            double y_offset = 0, double z_offset = 0);
};