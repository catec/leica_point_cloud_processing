#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

class ScanToPointCloud {
     public:
        ScanToPointCloud();
        void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan);

     private:
        ros::NodeHandle _node;
        laser_geometry::LaserProjection _projector;
        tf::TransformListener _tfListener;

        pcl::PointCloud<pcl::PointXYZ> _total_cloud; // complete point cloud from scene
        bool init_cloud = true; // attach header info and more to _total_cloud in first iteration

        ros::Publisher _pub;
        ros::Subscriber _sub;
};

void ScanToPointCloud::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud,total_cloud_msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl; // pointer to save ros scan info and updata _total_cloud
    cloud_pcl.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // Do transform
    if(!_tfListener.waitForTransform(
        scan->header.frame_id,
        "/world",
        scan->header.stamp + ros::Duration().fromSec(scan->ranges.size()*scan->time_increment),
        ros::Duration(1.0)))
    {
        ROS_ERROR("e");
        return;
    }

    _projector.transformLaserScanToPointCloud("/world", *scan, cloud, _tfListener);
    
    // Convert to PCL is necessary to add new scan data to _total_cloud
    pcl::fromROSMsg(cloud, *cloud_pcl);
    
    if (init_cloud)
    {
        _total_cloud = *cloud_pcl;
        init_cloud = false;
    }
    
    _total_cloud += *cloud_pcl;
    
    // Convert again to ros msg and publish
    pcl::toROSMsg(_total_cloud,total_cloud_msg);
    _pub.publish(total_cloud_msg);
}

ScanToPointCloud::ScanToPointCloud(){
        _sub = _node.subscribe<sensor_msgs::LaserScan> ("/c5/laser/scan", 100, &ScanToPointCloud::scanCb, this);
        _pub = _node.advertise<sensor_msgs::PointCloud2> ("/cloud", 100, false);
        // _tfListener.setExtrapolationLimit(ros::Duration(0.1));
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ScanToPointCloud");
    ROS_INFO("tf from laserscan to pointcloud");

    ScanToPointCloud filter;

    ros::spin();

    return 0;
}