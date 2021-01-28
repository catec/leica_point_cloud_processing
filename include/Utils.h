/**
 * @file Utils.h
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
#ifndef _UTILS_H
#define _UTILS_H

#include "ros/ros.h"
#include "ros/package.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>

#include <Viewer.h>

extern const std::string TARGET_CLOUD_TOPIC;
extern const std::string SOURCE_CLOUD_TOPIC;

class Utils
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
    typedef pcl::PointCloud<pcl::Normal> PointCloudNormal;

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
    static bool getNormals(PointCloudRGB::Ptr& cloud, double normal_radius, PointCloudNormal::Ptr& normals);

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
     * @brief Check whether cloud normals contains data and is not empty.
     * 
     * @param normals 
     * @return true 
     * @return false 
     */
    static bool isValidCloud(PointCloudNormal::Ptr normals);

    /**
     * @brief Check whether mesh contains data and is not empty.
     * 
     * @param mesh 
     * @return true 
     * @return false 
     */
    static bool isValidMesh(pcl::PolygonMesh::Ptr mesh);

    /**
     * @brief Check whether PointCloud2 contains data and is not empty.
     *
     * @param cloud_msg
     * @return true
     * @return false
     */
    static bool isValidCloudMsg(const sensor_msgs::PointCloud2& cloud_msg);

    /**
     * @brief Check whether transform contains valid data and is not NaN.
     * 
     * @param transform 
     * @return true 
     * @return false 
     */
    static bool isValidTransform(Eigen::Matrix4f transform);

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
    static void cloudToROSMsg(PointCloudRGB::Ptr cloud, sensor_msgs::PointCloud2& cloud_msg, const std::string& frameid="world");
    
    /**
     * @brief Convert XYZRGB cloud to PointCloud2.
     *
     * @param[in] cloud
     * @param[out] cloud_msg
     */
    static void cloudToROSMsg(const pcl::PCLPointCloud2& cloud, sensor_msgs::PointCloud2& cloud_msg, const std::string& frameid="world");

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
     * @brief Print on console matrix values with matrix format.
     * 
     * @param matrix 
     * @param size 
     */
    static void printMatrix(const Eigen::Ref<const Eigen::MatrixXf>& matrix, const int size);

    /**
     * @brief Apply extract indices filter to input cloud with given indices.
     *
     * @param[in] cloud_in
     * @param[in] indices
     * @param[out] cloud_out
     */

    static void onePointCloud(PointCloudRGB::Ptr cloud, int size,
                              PointCloudRGB::Ptr one_point_cloud);                            

    /**
     * @brief Apply translation to Cloud. Values given in [m]
     * 
     * @param cloud_in 
     * @param cloud_out 
     * @param x_offset 
     * @param y_offset 
     * @param z_offset 
     */
    static void translateCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, 
                              double x_offset = 0, double y_offset = 0, double z_offset = 0);

    /**
     * @brief Apply rotation to cloud. Values given in [rad]
     * 
     * @param cloud_in 
     * @param cloud_out 
     * @param roll 
     * @param pitch 
     * @param yaw 
     */
    static void rotateCloud(PointCloudRGB::Ptr cloud_in, PointCloudRGB::Ptr cloud_out, 
                            double roll, double pitch, double yaw);

    /**
     * @brief Obtain normal values as a vector
     * 
     * @param normal 
     * @param idx 
     * @param vector 
     */
    static void getVectorFromNormal(PointCloudNormal::Ptr normal, double idx,
                                    Eigen::Vector3f& vector);                          

    /**
     * @brief Check if both matrix have a coinciden row
     * 
     * @param source_m 
     * @param target_m 
     * @param source_indx 
     * @param target_indx 
     * @return true 
     * @return false 
     */
    static bool searchForSameRows(Eigen::MatrixXd source_m, Eigen::MatrixXd target_m,
                                  std::vector<int>& source_indx, std::vector<int>& target_indx);

    /**
     * @brief Check if given vectors are equal with a given tolerance (threshold).
     * 
     * @param v1 
     * @param v2 
     * @param threshold 
     * @return true 
     * @return false 
     */
    static bool areSameVectors(const Eigen::VectorXd v1, 
                              const Eigen::VectorXd v2, 
                              double threshold);

    /**
     * @brief Find value on vector with given threshold. Returns index vector of value found. If not found returns -1.
     * 
     * @param value 
     * @param v 
     * @param threshold 
     * @return int 
     */
    static int findOnVector(double value, 
                            const Eigen::VectorXd v, 
                            double threshold);

    /**
     * @brief Find vector as row on matrix with given threshold. Returns index of matrix row. If not found returns -1.
     * 
     * @param v 
     * @param m 
     * @param threshold 
     * @return int 
     */
    static int findOnMatrix(const Eigen::VectorXd v, 
                            const Eigen::MatrixXd m,  
                            double threshold); 

    /**
     * @brief Supress row from matrix
     * 
     * @param matrix 
     * @param row 
     */
    static void extractRowFromMatrix(Eigen::MatrixXd& matrix, int row);   

    /**
     * @brief Supress col from matrix
     * 
     * @param matrix 
     * @param col 
     */
    static void extractColFromMatrix(Eigen::MatrixXd& matrix, int col);  
};

#endif
