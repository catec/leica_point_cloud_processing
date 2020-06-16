#include "ros/ros.h"
#include "ros/package.h"
// #include <sensor_msgs/PointCloud2.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h> 
#include <pcl/features/normal_3d.h>
// #include <pcl/io/vtk_lib_io.h>
// #include <pcl/visualization/pcl_visualizer.h>


class Utils {
    public:
        Utils();
        ~Utils() {};

        std::string _pc_path;

        std::string getPCpath();
        static bool getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                               double normal_radius,
                               pcl::PointCloud<pcl::Normal>::Ptr &normals);
        static void cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                                  int R, int G, int B);
        static double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        static double computeCloudResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        static void printTransform(Eigen::Matrix4f transform);
};