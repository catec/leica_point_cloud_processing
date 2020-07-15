// cad_to_pointcloud.h
#pragma once
#ifndef _CAD_TO_POINTCLOUD_H
#define _CAD_TO_POINTCLOUD_H

#include <utils.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>

#endif


/**
 * @brief This class is used to obtain pcl::PointCloud from CAD file.
 *        Supported formats: .OBJ
 * 
 */
class CADToPointCloud {

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


public:

    /**
     * @brief Construct a new CADToPointCloud object
     * 
     */
    CADToPointCloud();

    /**
     * @brief Construct a new CADToPointCloud object
     * 
     * @param[in] cad_path
     * @param[in] cad_file Supported formats: .OBJ
     * @param[out] cloud XYZ
     */
    CADToPointCloud(std::string cad_path,
                    std::string cad_file, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
    
    /**
     * @brief Construct a new CADToPointCloud object
     * 
     * @param[in] cad_file_path Supported formats: .OBJ
     * @param[out] cloud XYZRGB
     */
    CADToPointCloud(std::string cad_file_path, 
                    PointCloudRGB::Ptr cloud);
    
    /**
     * @brief Destroy the CADToPointCloud object
     * 
     */
    ~CADToPointCloud() {};


    /** @brief pcl::PolygonMesh to store mesh from CAD file. */
    pcl::PolygonMesh _CAD_mesh;

    /** @brief pcl::Pointcloud to store XYZ cloud from CAD file. */
    pcl::PointCloud<pcl::PointXYZ>::Ptr _CAD_cloud{new pcl::PointCloud<pcl::PointXYZ>};
    
    /** @brief PointCloud2 to store XYZ cloud from CAD file. Ready to be published. */
    sensor_msgs::PointCloud2 _CAD_cloud_msg; 
    
    /** @brief Absolute path to pointcloud folder. */
    std::string _pc_path;
    

    /**
     * @brief Set path to pointcloud folder.
     * 
     * @param path 
     */
    void setPCpath(std::string path);

    /**
     * @brief Load specified file into CAD_mesh object. 
     *        Return -1 if error.
     * 
     * @param cad_file_path Supported formats: .OBJ
     * @return int 
     */
    int CADToMesh(std::string cad_file_path);

    /**
     * @brief Convert mesh to CAD_cloud object.
     * 
     * @param mesh 
     * @return int 
     */
    int MeshToPointCloud(pcl::PolygonMesh mesh);

    /**
     * @brief Convert mesh to CAD_cloud_msg object.
     * 
     * @param mesh 
     * @return int 
     */
    int MeshToROSPointCloud(pcl::PolygonMesh mesh);


private:

    /** @brief Extracted from Point Cloud Library (PCL)  --> pcl/tools/mesh_sampling.cpp
     *         \n Copyright (c) 2010-2011, Willow Garage, Inc. */
    void uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> &cloud_out);

    /** @brief Extracted from Point Cloud Library (PCL)  --> pcl/tools/mesh_sampling.cpp
     *         \n Copyright (c) 2010-2011, Willow Garage, Inc. */
    void randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea, Eigen::Vector4f &p);    

    /** @brief Extracted from Point Cloud Library (PCL)  --> pcl/tools/mesh_sampling.cpp
     *         \n Copyright (c) 2010-2011, Willow Garage, Inc. */
    void randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f &p);

    /** @brief Extracted from Point Cloud Library (PCL)  --> pcl/tools/mesh_sampling.cpp
     *         \n Copyright (c) 2010-2011, Willow Garage, Inc. */
    double uniform_deviate(int seed);   
};