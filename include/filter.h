#include "ros/ros.h"
#include "ros/package.h"
#include <pcl_ros/point_cloud.h> 
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/search/kdtree.h>
#include "pcl/common/angles.h"
#include "pcl/segmentation/sac_segmentation.h"
// #include <utils.h>
#include <initial_alignment.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class Filter {
    public:
        Filter() {};
        ~Filter() {};

        double _leaf_size, _noise_filter_threshold, _floor_filter_threshold;
        ros::Publisher pub;

        void downsampleCloud(PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_downsampled, const Eigen::Vector4f leaf_size);
        void filter_noise(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
        void filter_floor(double threshold, PointCloudRGB::Ptr cloud, PointCloudRGB::Ptr cloud_filtered);
};