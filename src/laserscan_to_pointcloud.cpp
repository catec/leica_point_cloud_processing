#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        pcl::PointCloud<pcl::PointXYZ> total_cloud_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;

        // sensor_msgs::PointCloud2 total_cloud_;
};

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud,total_cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl;

    cloud_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>);

    projector_.transformLaserScanToPointCloud("/world", *scan, cloud, tfListener_);
    
    // pcl::fromROSMsg(cloud, *cloud_pcl);
    // total_cloud_ += *cloud_pcl;
    
    // pcl::toROSMsg(total_cloud_,total_cloud_msg);
    point_cloud_publisher_.publish(cloud);
}

My_Filter::My_Filter(){
        scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/laser/scan", 100, &My_Filter::scanCallback, this);
        point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        // tfListener_.setExtrapolationLimit(ros::Duration(0.1));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_filter");
    ROS_INFO("tf from laserscan to pointcloud");

    My_Filter filter;

    ros::spin();

    return 0;
}