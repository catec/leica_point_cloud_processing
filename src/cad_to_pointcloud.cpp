#include <cad_to_pointcloud.h>


CADToPointCloud::CADToPointCloud()
{
    _pc_path = getPCpath();
    RED = {255,0,0};
    GREEN = {0,255,0};
    BLUE = {0,0,255};
}

CADToPointCloud::CADToPointCloud(std::string cad_file, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, bool big_file)
{
    _pc_path = getPCpath();
    CADToMesh(cad_file); // here we get _CAD_mesh

    if (big_file) MeshToPointCloud(_CAD_mesh); // here we get _CAD_cloud
    else MeshToPointCloud(cad_file); // here we get _CAD_cloud

    // MeshToPointCloud(_CAD_mesh); // here we get _CAD_cloud

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

void CADToPointCloud::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pc_color color)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cloud, color.r, color.g, color.b); 
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pointcloud Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_rgb,"cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    
    while (!viewer->wasStopped ()){
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    } 
}

void CADToPointCloud::visualizePointCloudAndNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                                    pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                    pc_color color)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cloud, color.r, color.g, color.b); 

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Pointcloud and Normals Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_rgb,"cloud");
    viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,normals,10,0.02,"normals");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,10);
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

void CADToPointCloud::MeshToPointCloud(std::string filename)
{
    // TODO
    // deshacer esta forma tan cutre
    std::string inputfile = _pc_path+filename;
    std::string outputfile = _pc_path+"tmp.pcd";

    std::string cmd = "pcl_mesh_sampling "+ inputfile + " " + outputfile + " -n_samples 500000";
    system(cmd.c_str());

    pcl::io::loadPCDFile<pcl::PointXYZ> (outputfile, *_CAD_cloud);

    cmd = "rm " + outputfile;
    system(cmd.c_str());
}

void CADToPointCloud::MeshToPointCloud(pcl::PolygonMesh mesh)
{
    pcl::fromPCLPointCloud2(mesh.cloud, *_CAD_cloud);
    // TODO esta funcion esta muy limitada. 
    // Hay que intentar reproducir el comportamiento de este programa:
    // pcl_mesh_sampling input.obj output.pcd    

    // visualizePointCloud(_CAD_cloud);
}

void CADToPointCloud::MeshToROSPointCloud(pcl::PolygonMesh mesh)
{
    pcl_conversions::fromPCL( mesh.cloud, _CAD_cloud_msg);

    // Hay una función de pcl_ros muy util:
    // rosrun pcl_ros pcd_to_pointcloud input.pcd periodo _frame_id:=/world
}


std::string CADToPointCloud::getPCpath()
{
    std::string pkg_path = ros::package::getPath("leica_scanstation");

	_pc_path = pkg_path + "/pointclouds/";
	return _pc_path;
}