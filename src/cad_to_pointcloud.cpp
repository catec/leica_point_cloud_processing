#include "ros/ros.h"
#include "ros/package.h"
// #include <tf/transform_listener.h>
#include "pcl_conversions/pcl_conversions.h"
// #include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/tools/mesh2pcd.h>
// #include <pcl/filters/voxel_grid.h>
#include <pcl/io/


class CADToPointCloud {
    public:
        CADToPointCloud();

        pcl::PointCloud<pcl::PointXYZ> _cloud, _mesh_cloud;
        // sensor_msgs::PointCloud2 _mesh_cloud_msg; 
        std::string _hello,_pc_path;
        void testing(std::string filename);
        void visualizeMesh(pcl::PolygonMesh mesh);
        void visualizePointCloud(pcl::PointCloud<pcl::PointXYZ> pc);
        std::string getPCpath();
};


CADToPointCloud::CADToPointCloud(){
    _hello = "Helloworld";
    _pc_path = getPCpath();
}

void CADToPointCloud::visualizeMesh(pcl::PolygonMesh mesh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void CADToPointCloud::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ> pc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(&pc);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud(cloud);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void CADToPointCloud::testing(std::string filename)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile(_pc_path+filename,mesh);

    pcl::fromPCLPointCloud2(mesh.cloud,_mesh_cloud);

    visualizePointCloud(_mesh_cloud);

    // pcl_conversions::fromPCL( mesh.cloud, _mesh_cloud_msg );
}


std::string CADToPointCloud::getPCpath()
{
    std::string pkg_path = ros::package::getPath("leica_scanstation");

	_pc_path = pkg_path + "/pointclouds/";
	return _pc_path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "CADToPointCloud");

    CADToPointCloud cad_to_pointcloud;

    ROS_INFO(cad_to_pointcloud._hello.c_str());

    cad_to_pointcloud.testing("untitled.obj");

    ros::spin();

    return 0;
}