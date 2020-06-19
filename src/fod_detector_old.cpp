#include <fod_detector.h>

FODDetector::FODDetector() 
{
    _voxel_resolution = 0;
}

double FODDetector::getVoxelResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    if(_voxel_resolution == 0)
    {
        computeVoxelResolution(cloud);
    }
    return _voxel_resolution;
}

void FODDetector::computeVoxelResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZ> tree;
    tree.setInputCloud(cloud);

    for (std::size_t i = 0; i < cloud->size (); ++i)
    {
        if (! std::isfinite ((*cloud)[i].x))
        {
        continue;
        }
        //Considering the second neighbor since the first is the point itself.
        nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
        if (nres == 2)
        {
        res += sqrt (sqr_distances[1]);
        ++n_points;
        }
    }
    if (n_points != 0)
    {
        res /= n_points;
    }
    _voxel_resolution = res*3; // 3 times higher to cover more than two neighbors    
}

void FODDetector::booleanDiffBetweenClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    // Compute B-A in clouds
    ROS_INFO("Creating octree with resolution: %f",_voxel_resolution);
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(_voxel_resolution);
    octree.setInputCloud(cloud_a);
    octree.addPointsFromInputCloud();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    octree.setInputCloud(cloud_b);
    octree.addPointsFromInputCloud();

    ROS_INFO("Extracting differences");
    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>); // Diferentiated points
    octree.getPointIndicesFromNewVoxels(*indices);

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud_b);
    extract_indices_filter.setIndices(indices);
    extract_indices_filter.filter(*output);
}

void FODDetector::clusterPossibleFODs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      std::vector<pcl::PointIndices> &cluster_indices)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(_voxel_resolution);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
}

int FODDetector::clusterIndicesToROSMsg(std::vector<pcl::PointIndices> cluster_indices,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         std::vector<sensor_msgs::PointCloud2> &cluster_msg_array)
{
    int n_fods=0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::string cluster_name = "/fod"+std::to_string(n_fods);
        n_fods++;

        // apply color to publish
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloudToXYZRGB(cloud_cluster,cloud_rgb,255,0,128);

        sensor_msgs::PointCloud2 cluster_msg;
        pcl::toROSMsg(*cloud_rgb,cluster_msg);
        cluster_msg.header.frame_id = "world";
        cluster_msg.header.stamp = ros::Time::now();
        cluster_msg_array.push_back(cluster_msg);
    }
    return n_fods;
}

void FODDetector::cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                                int R, int G, int B)
{
    pcl::copyPointCloud(*cloud,*cloud_rgb);
    for (size_t i = 0; i < cloud_rgb->points.size(); i++)
    {
        cloud_rgb->points[i].r = R;
        cloud_rgb->points[i].g = G;
        cloud_rgb->points[i].b = B;
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "FODDetector");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("open file");
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio_cad.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cad_pc);
    f = cad_to_pointcloud._pc_path + "conjunto_estranio_fod_aligned.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

    Viewer leica_viewer;
    leica_viewer.addPCToViewer(cad_pc,leica_viewer.BLUE,"cad");
    leica_viewer.addPCToViewer(scan_pc,leica_viewer.PINK,"scan");

    FODDetector fod_detector;

    double voxel_resolution = fod_detector.getVoxelResolution(scan_pc);
    pcl::PointCloud<pcl::PointXYZ>::Ptr diff_pc(new pcl::PointCloud<pcl::PointXYZ>);
    fod_detector.booleanDiffBetweenClouds(cad_pc, scan_pc, diff_pc);

    if (diff_pc->size()<=0) 
    {
        ROS_ERROR("No differences found.");
        return 0;
    }

    leica_viewer.setPointSize(3);
    leica_viewer.addPCToViewer(diff_pc,leica_viewer.WHITE,"diff");
    leica_viewer.deletePCFromViewer("cloud");
    leica_viewer.deletePCFromViewer("scan");

    ROS_INFO("Getting possible fods");
    std::vector<pcl::PointIndices>cluster_indices; //This is a vector of cluster
    fod_detector.clusterPossibleFODs(diff_pc, cluster_indices);
    
    // iterate through cluster_indices to create a pc for each cluster
    std::vector<sensor_msgs::PointCloud2> cluster_msg_array;
    int num_of_fods = fod_detector.clusterIndicesToROSMsg(cluster_indices, diff_pc, cluster_msg_array);

    ROS_INFO("Detected %d objects",num_of_fods);
    ros::Publisher npub = nh.advertise<std_msgs::Int16>("/num_of_fods", 1);
    std_msgs::Int16 msg;
    msg.data = num_of_fods;
    npub.publish(msg);

    ROS_INFO("Publishing on topics /fodX");
    // publish each fod in a separated topic
    while(ros::ok())
    {
        for(int i=0; i<num_of_fods; i++) // temporary solution
        {
            std::string topic_name = "/fod"+std::to_string(i);
            ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);

            pub.publish(cluster_msg_array[i]);
        }
    }

    return 0;
}