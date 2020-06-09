#include "ros/ros.h"
// #include "ros/package.h"
// #include <sensor_msgs/PointCloud2.h>
// #include "pcl_conversions/pcl_conversions.h"
// #include <pcl_ros/point_cloud.h> 
// #include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>


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
        void deletePCFromViewer(std::string name);
        void addNormalsToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PointCloud<pcl::Normal>::Ptr normals,
                                std::string name);
        void addCorrespondencesToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                        pcl::CorrespondencesPtr correspondences);
        
        void resetViewer();

        static void visualizeMesh(pcl::PolygonMesh mesh);
        static void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};