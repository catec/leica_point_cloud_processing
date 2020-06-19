#include <stdlib.h> 
#include <string>
#include <pcl_ros/point_cloud.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
// #include <pcl/surface/organized_fast_mesh.h>
// #include <pcl/features/normal_3d.h>
// #include <pcl/surface/mls.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/surface/gp3.h>
// #include <pcl_ros/point_cloud.h> 
// #include <pcl/filters/extract_indices.h>
// #include <pcl/octree/octree_pointcloud_changedetector.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Int16.h>
#include <viewer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class FODDetector 
{
    public:
        FODDetector(double resolution);
        ~FODDetector() {};

        void clusterPossibleFODs(PointCloudRGB::Ptr cloud,
                                 std::vector<pcl::PointIndices> &cluster_indices);
        int clusterIndicesToROSMsg(std::vector<pcl::PointIndices> cluster_indices,
                                   PointCloudRGB::Ptr cloud,
                                   std::vector<sensor_msgs::PointCloud2> &cluster_msg_array);

    private:
        double _voxel_resolution;
};