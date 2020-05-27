#include <cad_to_pointcloud.h>


CADToPointCloud::CADToPointCloud()
{
    _pc_path = getPCpath();
    RED = {255,0,0};
    GREEN = {0,255,0};
    BLUE = {0,0,255};
    PINK = {255,0,128};
    ORANGE = {255,128,0};
    WHITE = {255,255,255};
}

CADToPointCloud::CADToPointCloud(std::string cad_file, pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, bool big_file)
{
    _pc_path = getPCpath();
    RED = {255,0,0};
    GREEN = {0,255,0};
    BLUE = {0,0,255};
    PINK = {255,0,128};
    ORANGE = {255,128,0};
    WHITE = {255,255,255};

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
    resetVisualizer();
    _viewer->setBackgroundColor (0, 0, 0);
    _viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_rgb,"cloud");
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2);
    _viewer->addCoordinateSystem (1.0);
    _viewer->initCameraParameters ();
    
    while (!_viewer->wasStopped ()){
        _viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    } 
}

void CADToPointCloud::resetVisualizer()
{
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
}


void CADToPointCloud::addNormalsToVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                             pcl::PointCloud<pcl::Normal>::Ptr normals,
                                             std::string name)
{
    ROS_INFO("Add normals to cloud in visualizer");
    _viewer->resetStoppedFlag();
    _viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,normals,1,0.02,name);
    while (!_viewer->wasStopped ()){
        _viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    } 
}

void CADToPointCloud::addPCToVisualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pc_color color, std::string name)
{
    ROS_INFO("Add cloud in visualizer");
    _viewer->resetStoppedFlag();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cloud, color.r, color.g, color.b); 
    _viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_rgb,name);
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2,name);
    while (!_viewer->wasStopped ()){
        _viewer->spinOnce (100);
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