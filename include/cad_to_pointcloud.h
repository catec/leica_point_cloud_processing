#include "ros/ros.h"
#include "ros/package.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>

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
                        pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, 
                        bool big_file);
        CADToPointCloud(std::string file_path, 
                        PointCloudRGB::Ptr &cloud, 
                        bool big_file);
        ~CADToPointCloud() {};

        pcl::PolygonMesh _CAD_mesh;
        pcl::PointCloud<pcl::PointXYZ>::Ptr _CAD_cloud{new pcl::PointCloud<pcl::PointXYZ>};
        sensor_msgs::PointCloud2 _CAD_cloud_msg; 
        std::string _pc_path;
        
        void setPCpath(std::string path);
        int CADToMesh(std::string file_path);
        int MeshToPointCloud(std::string file_path);
        int MeshToPointCloud(pcl::PolygonMesh mesh);
        int MeshToROSPointCloud(pcl::PolygonMesh mesh);
};