#include <cad_to_pointcloud.h>


CADToPointCloud::CADToPointCloud()
{
    _pc_path = getPCpath();
}

CADToPointCloud::CADToPointCloud(std::string cad_file, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{
    _pc_path = getPCpath();
    CADToMesh(cad_file); // here we get _CAD_mesh
    MeshToPointCloud(_CAD_mesh); // here we get _CAD_cloud

    pointcloud = _CAD_cloud;
}

void CADToPointCloud::visualizeMesh(pcl::PolygonMesh mesh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Mesh Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPolygonMesh(mesh,"meshes",0);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

void CADToPointCloud::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pointcloud Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud(cloud);
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}


void CADToPointCloud::CADToMesh(std::string filename)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile(_pc_path+filename,mesh);
    
    _CAD_mesh = mesh;

    // visualizeMesh(_CAD_mesh);
}

void CADToPointCloud::MeshToPointCloud(pcl::PolygonMesh mesh)
{
    ROS_INFO("MeshToPointCloud");
    pcl::fromPCLPointCloud2(mesh.cloud, *_CAD_cloud);

    ROS_INFO("MeshToPointCloud VISUALIZING");
    visualizePointCloud(_CAD_cloud);
}

void CADToPointCloud::MeshToROSPointCloud(pcl::PolygonMesh mesh)
{
    pcl_conversions::fromPCL( mesh.cloud, _CAD_cloud_msg);
}


std::string CADToPointCloud::getPCpath()
{
    std::string pkg_path = ros::package::getPath("leica_scanstation");

	_pc_path = pkg_path + "/pointclouds/";
	return _pc_path;
}