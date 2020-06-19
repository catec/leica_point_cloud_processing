// viewer.h
#pragma once
#ifndef _VIEWER_H
#define _VIEWER_H

#include "ros/ros.h"
#include <pcl/visualization/pcl_visualizer.h>

#endif 

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
        
        void addPCToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pc_color color,std::string name);
        void addPCToViewer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name);
        void deletePCFromViewer(std::string name);
        void addNormalsToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr normals,
                                std::string name);
        void addCorrespondencesToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                        pcl::CorrespondencesPtr correspondences);
        
        void resetViewer();


        static void visualizeMesh(pcl::PolygonMesh mesh);
        template <typename PointT>
        static void visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
        {
            ROS_INFO("Display cloud in Viewer");
            ROS_WARN("Press (X) on viewer to continue");
            boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer(new pcl::visualization::PCLVisualizer ("Pointcloud viewer"));
            pc_viewer->setBackgroundColor(0, 0, 0);
            pc_viewer->addPointCloud<PointT>(cloud,"cloud");
            pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
            pc_viewer->addCoordinateSystem(1.0);
            pc_viewer->initCameraParameters();

            while (!pc_viewer->wasStopped()){
                pc_viewer->spinOnce(100);
                boost::this_thread::sleep(boost::posix_time::microseconds (100000));
            } 
        }
};