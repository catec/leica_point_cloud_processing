#include <Viewer.h>


Viewer::Viewer()
{
    _point_size = 2;
    setColors();
    // configViewer();
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
    // _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size);
    _viewer->addCoordinateSystem(1.0);
    _viewer->initCameraParameters();
}

void Viewer::setPointSize(int point_size)
{
    _point_size = point_size;
    _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size);
}

void Viewer::resetViewer()
{
    _viewer->removeAllCoordinateSystems();
    _viewer->removeAllPointClouds();
    _viewer->removeAllShapes();
}

void Viewer::loopViewer()
{
    while (!_viewer->wasStopped())
    {
        _viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    } 
}

template <typename PointT>
inline void Viewer::addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                  const pc_color &color, 
                                  const std::string &name)
{
    configViewer();
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_rgb(cloud, color.r, color.g, color.b);
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
        ROS_INFO("Update cloud in Viewer");
        _viewer->updatePointCloud(cloud, cloud_rgb, name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size, name);
    }
    else
    {
        ROS_INFO("Add cloud in Viewer");
        _viewer->addPointCloud<PointT>(cloud, cloud_rgb, name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size, name);
    }
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();
}

template <typename PointT>
inline void Viewer::addPCToViewer(typename pcl::PointCloud<PointT>::Ptr cloud, 
                                  const std::string &name)
{
    configViewer();
    _viewer->resetStoppedFlag();

    if (_viewer->contains(name))
    {
        ROS_INFO("Update cloud in Viewer");
        _viewer->updatePointCloud(cloud, name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size, name);
    }
    else
    {
        ROS_INFO("Add cloud in Viewer");
        _viewer->addPointCloud<PointT>(cloud, name);
        _viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, _point_size, name);
    }
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();
}

void Viewer::deletePCFromViewer(const std::string &name)
{
    _viewer->resetStoppedFlag();

    ROS_INFO("Remove cloud from Viewer");
    _viewer->removePointCloud(name);
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();
}

template <typename PointT, typename CloudT>
inline void Viewer::addNormalsToViewer(typename pcl::PointCloud<PointT>::Ptr cloud,
                                       typename pcl::PointCloud<CloudT>::Ptr normals,
                                       const std::string &name)
{
    _viewer->resetStoppedFlag();

    ROS_INFO("Add normals to cloud in Viewer");
    _viewer->addPointCloudNormals<PointT, CloudT>(cloud, normals, 1, 0.02, name);
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();
}

template <typename PointT>
inline void Viewer::addCorrespondencesToViewer(typename pcl::PointCloud<PointT>::Ptr source_cloud,
                                               typename pcl::PointCloud<PointT>::Ptr target_cloud,
                                               pcl::CorrespondencesPtr correspondences)
{
    _viewer->resetStoppedFlag();

    ROS_INFO("Add correspondences between clouds in Viewer");
    _viewer->addCorrespondences<PointT>(source_cloud, target_cloud, *correspondences);
    ROS_WARN("Press (X) on viewer to continue");

    loopViewer();

    _viewer->removeCorrespondences();
}

void Viewer::keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
    if (event.getKeySym() == "space" && event.keyDown())
        _pressed_space = true;
}

void Viewer::checkForSpaceKeyPressed()
{
    _viewer->registerKeyboardCallback(&Viewer::keyboardCallback,*this);
}

void Viewer::visualizeMesh(pcl::PolygonMesh::Ptr mesh)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> 
        mesh_viewer(new pcl::visualization::PCLVisualizer("Mesh mesh_viewer"));
    ROS_INFO("Display mesh in Viewer");

    mesh_viewer->setBackgroundColor(0, 0, 0);
    mesh_viewer->addPolygonMesh(*mesh, "meshes", 0);
    mesh_viewer->addCoordinateSystem(1.0);
    mesh_viewer->initCameraParameters();
    ROS_WARN("Press (X) on viewer to continue");

    while (!mesh_viewer->wasStopped())
    {
        mesh_viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

template <typename PointT>
void Viewer::visualizePointCloud(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    boost::shared_ptr<pcl::visualization::PCLVisualizer> 
        pc_viewer(new pcl::visualization::PCLVisualizer("Pointcloud viewer"));
    ROS_INFO("Display cloud in Viewer");

    pc_viewer->setBackgroundColor(0, 0, 0);
    pc_viewer->addPointCloud<PointT>(cloud, "cloud");
    pc_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,3);
    pc_viewer->addCoordinateSystem(1.0);
    pc_viewer->initCameraParameters();
    ROS_WARN("Press (X) on viewer to continue");

    while (!pc_viewer->wasStopped())
    {
        pc_viewer->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    } 
}