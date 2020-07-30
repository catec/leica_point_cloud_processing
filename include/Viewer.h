// viewer.h
#pragma once
#ifndef _VIEWER_H
#define _VIEWER_H

#include "ros/ros.h"
#include <pcl/visualization/pcl_visualizer.h>

#endif 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class Viewer 
{
public:
    Viewer();
    ~Viewer() {};

    struct pc_color 
    { 
        int r,g,b; 
    };
    pc_color RED,GREEN,BLUE,PINK,ORANGE,WHITE;
    double _point_size;
    bool _pressed_space;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> _viewer{new pcl::visualization::PCLVisualizer ("Pointcloud Viewer")};

    void setColors();
    void configViewer();
    void loopViewer();
    void setPointSize(int point_size);
    void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* nothing);
    void checkForSpaceKeyPressed();
    
    template <typename PointT>
    void addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, 
                       const pc_color &color, 
                       const std::string &name);

    template <typename PointT>
    void addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud,
                       const std::string &name);

    void deletePCFromViewer(const std::string &name);

    template <typename PointT, typename CloudT>
    void addNormalsToViewer(typename pcl::PointCloud<PointT>::Ptr cloud,
                            typename pcl::PointCloud<CloudT>::Ptr normals,
                            const std::string &name);

    template <typename PointT>
    void addCorrespondencesToViewer(typename pcl::PointCloud<PointT>::Ptr source_cloud,
                                    typename pcl::PointCloud<PointT>::Ptr target_cloud,
                                    pcl::CorrespondencesPtr correspondences);
    
    void resetViewer();


    static void visualizeMesh(pcl::PolygonMesh::Ptr mesh);

    template <typename PointT>
    static void visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
};