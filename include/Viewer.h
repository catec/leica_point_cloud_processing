// viewer.h
#pragma once
#ifndef _VIEWER_H
#define _VIEWER_H

#include "ros/ros.h"
#include <pcl/visualization/pcl_visualizer.h>

#endif 

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class Viewer {
    public:
        Viewer();
        ~Viewer() {};

        struct pc_color { int r,g,b; };
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
        
        void addPCToViewer(PointCloudXYZ::Ptr cloud, pc_color color, const std::string &name);
        void addPCToViewer(PointCloudRGB::Ptr cloud, const std::string &name);
        void deletePCFromViewer(const std::string &name);
        void addNormalsToViewer(PointCloudRGB::Ptr cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr normals,
                                const std::string &name);
        void addCorrespondencesToViewer(PointCloudRGB::Ptr source_cloud,
                                        PointCloudRGB::Ptr target_cloud,
                                        pcl::CorrespondencesPtr correspondences);
        
        void resetViewer();


        static void visualizeMesh(pcl::PolygonMesh mesh);
        template <typename PointT>
        static void visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
};