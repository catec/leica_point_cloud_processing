#include "ros/ros.h"
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
// #include "pcl_conversions/pcl_conversions.h"
// #include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>

/** Example use:
 *      
 *      pcl::PointCloud<pcl::PointXYZ> pc;
 *      
 *      CADToPointCloud cad_to_pointcloud = CADToPointCloud("untitled.obj", pc);
 **/

class CADToPointCloud {
    public:
        CADToPointCloud();
        CADToPointCloud(std::string cad_file, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, bool big_file);
        ~CADToPointCloud() {};

        pcl::PolygonMesh _CAD_mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _CAD_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        sensor_msgs::PointCloud2 _CAD_cloud_msg; 
        std::string _pc_path;
        void CADToMesh(std::string filename);
        void MeshToPointCloud(std::string filename);
        void MeshToPointCloud(pcl::PolygonMesh mesh);
        void MeshToROSPointCloud(pcl::PolygonMesh mesh);
        void visualizeMesh(pcl::PolygonMesh mesh);
        void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        std::string getPCpath();
};