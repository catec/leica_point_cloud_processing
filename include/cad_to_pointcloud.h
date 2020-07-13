// cad_to_pointcloud.h
#pragma once
#ifndef _CAD_TO_POINTCLOUD_H
#define _CAD_TO_POINTCLOUD_H

#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>

#endif

/** Example use:
 *      
 *      pcl::PointCloud<pcl::PointXYZ> pc;
 *      
 *      CADToPointCloud cad_to_pointcloud = CADToPointCloud("untitled.obj", pc);
 **/

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class CADToPointCloud {
    public:
        CADToPointCloud();
        CADToPointCloud(std::string pointcloud_path,
                        std::string cad_file, 
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
        CADToPointCloud(std::string cad_file_path, 
                        PointCloudRGB::Ptr cloud);
        ~CADToPointCloud() {};

        pcl::PolygonMesh _CAD_mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _CAD_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        sensor_msgs::PointCloud2 _CAD_cloud_msg; 
        std::string _pc_path;
        
        void setPCpath(std::string path);
        int CADToMesh(std::string cad_file_path);
        int MeshToPointCloud(pcl::PolygonMesh mesh);
        int MeshToROSPointCloud(pcl::PolygonMesh mesh);

    private:
        /* 
            Copyright of what's below:   pcl / tools / mesh_sampling.cpp
        */
        void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> &cloud_out);
        void randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea, Eigen::Vector4f &p);    
        void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f &p);
        double uniform_deviate(int seed);   
};