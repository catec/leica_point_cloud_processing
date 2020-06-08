#include <stdlib.h> 
#include <string>
#include <pcl_ros/point_cloud.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h> 
#include <pcl/filters/extract_indices.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Int16.h>
#include <cad_to_pointcloud.h>

class FODDetector 
{
    public:
        FODDetector();
        ~FODDetector() {};


        double getVoxelResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        void booleanDiffBetweenClouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_b,
                                      pcl::PointCloud<pcl::PointXYZ>::Ptr output);
        void clusterPossibleFODs(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 std::vector<pcl::PointIndices> &cluster_indices);
        int clusterIndicesToROSMsg(std::vector<pcl::PointIndices> cluster_indices,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                   std::vector<sensor_msgs::PointCloud2> &cluster_msg_array);
        void cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                           int R, int G, int B);

    private:
        double _voxel_resolution;
        
        void computeVoxelResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
};