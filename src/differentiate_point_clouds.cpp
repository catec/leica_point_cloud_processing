#include <stdlib.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <cad_to_pointcloud.h>

#define NORMAL_RADIUS 0.1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CloudToMesh");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);

    ROS_INFO("open file");
    CADToPointCloud cad_to_pointcloud = CADToPointCloud("conjunto_estranio_scan_transformed_reconstructed.obj", scan_pc, false);
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio_cad.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cad_pc);

    // cad_to_pointcloud.visualizePointCloud(cad_pc,cad_to_pointcloud.WHITE);
    // cad_to_pointcloud.addPCToVisualizer(scan_pc,cad_to_pointcloud.PINK,"scan");

    // Octree resolution - side length of octree voxels
    float resolution = 32.0f;
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(scan_pc);
    octree.addPointsFromInputCloud();

    // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    octree.switchBuffers();

    octree.setInputCloud(cad_pc);
    octree.addPointsFromInputCloud();

    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>); // Interest points
    octree.getPointIndicesFromNewVoxels(*indices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr diff_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(scan_pc);
    extract_indices_filter.setIndices(indices);
    extract_indices_filter.filter(*diff_pc);

    cad_to_pointcloud.visualizePointCloud(diff_pc,cad_to_pointcloud.BLUE);

    return 0;
}