#include <viewer.h>


Viewer::Viewer()
{
    _point_size = 2;
    setColors();
    configViewer();
}

void Viewer::setColors()
{
    RED = {255,0,0};
    GREEN = {0,255,0};
    BLUE = {0,0,255};
    PINK = {255,0,128};
    ORANGE = {255,128,0};
    WHITE = {255,255,255};
}

void Viewer::configViewer()
{
    _viewer->setBackgroundColor(0, 0, 0);
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,_point_size);
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
}

void Viewer::setPointSize(int point_size)
{
    _point_size = point_size;
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,_point_size);
}

void Viewer::resetViewer()
{
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
}

void Viewer::loopViewer()
{
    while (!_viewer->wasStopped()){
        _viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    } 
}

void Viewer::addPCToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pc_color color, std::string name)
{
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cloud, color.r, color.g, color.b); 
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
        ROS_INFO("Update cloud in Viewer");
        _viewer->updatePointCloud(cloud,cloud_rgb,name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,_point_size,name);
    }
    else
    {
        ROS_INFO("Add cloud in Viewer");
        _viewer->addPointCloud<pcl::PointXYZ>(cloud,cloud_rgb,name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,_point_size,name);
    }
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();
}

void Viewer::deletePCFromViewer(std::string name)
{
    _viewer->resetStoppedFlag();
    _viewer->removePointCloud(name);
    loopViewer();
}

void Viewer::addNormalsToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                             pcl::PointCloud<pcl::Normal>::Ptr normals,
                                             std::string name)
{
    ROS_INFO("Add normals to cloud in Viewer");
    ROS_WARN("Press (X) on viewer to continue");
    _viewer->resetStoppedFlag();
    _viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud,normals,1,0.02,name);
    loopViewer();
}

void Viewer::addCorrespondencesToViewer(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                                     pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                                     pcl::CorrespondencesPtr correspondences)
{
    ROS_INFO("Add correspondences between clouds in Viewer");
    ROS_WARN("Press (X) on viewer to continue");
    _viewer->resetStoppedFlag();
    _viewer->addCorrespondences<pcl::PointXYZ>(source_cloud,target_cloud,*correspondences);
    loopViewer();
    _viewer->removeCorrespondences();
}

void Viewer::keyboardCallback(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        _pressed_space = true;
}

void Viewer::checkForSpaceKeyPressed()
{
    _viewer->registerKeyboardCallback(&Viewer::keyboardCallback,*this);
}

void Viewer::visualizePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    ROS_INFO("Display cloud in Viewer");
    ROS_WARN("Press (X) on viewer to continue");
    boost::shared_ptr<pcl::visualization::PCLVisualizer> pc_viewer(new pcl::visualization::PCLVisualizer ("Pointcloud viewer"));
    pc_viewer->setBackgroundColor(0, 0, 0);
    pc_viewer->addPointCloud<pcl::PointXYZ>(cloud,"cloud");
    pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
    pc_viewer->addCoordinateSystem(1.0);
    pc_viewer->initCameraParameters();
    
    while (!pc_viewer->wasStopped()){
        pc_viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    } 
}

void Viewer::visualizeMesh(pcl::PolygonMesh mesh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> mesh_viewer(new pcl::visualization::PCLVisualizer ("Mesh mesh_viewer"));
    ROS_INFO("Display mesh in Viewer");
    mesh_viewer->setBackgroundColor(0, 0, 0);
    mesh_viewer->addPolygonMesh(mesh,"meshes",0);
    mesh_viewer->addCoordinateSystem(1.0);
    mesh_viewer->initCameraParameters();
    while (!mesh_viewer->wasStopped()){
        mesh_viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds (100000));
    }
}