#include "ros/ros.h"
#include "ros/package.h"
#include "std_srvs/Trigger.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl_ros/point_cloud.h>
#include <cad_to_pointcloud.h>
// #include <utils.h>
#include <filter.h>
// #include <initial_alignment.h>
#include <gicp_alignment.h>
#include <boolean_difference.h>
#include <viewer.h>
#include <fod_detector.h>


/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/

std::string TARGET_CLOUD_TOPIC = "/cad/cloud";
std::string SOURCE_CLOUD_TOPIC = "/scan/cloud";

// Pointclouds
PointCloudRGB::Ptr g_cad_pc(new PointCloudRGB);
PointCloudRGB::Ptr g_scan_pc(new PointCloudRGB);


// Flags to control program flow
bool new_cad_pc = false;
bool new_scan_pc = false;
bool iterate = false;
bool undo_last_iteration = false;
bool get_fods = false;


// Some parameters
int FREQ = 5; // Hz


bool gicpCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    iterate = true; 

    res.success = true;
    res.message = "iteration";
}

bool undoCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    undo_last_iteration = true; 

    res.success = true;
    res.message = "undo";
}

bool fodsCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    get_fods = true; 

    res.success = true;
    res.message = "fods";
}

void cadCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!new_cad_pc)
    {
        ROS_INFO("Get CAD cloud");
        // Save cloud
        pcl::fromROSMsg(*msg, *g_cad_pc);
        new_cad_pc = true;

        if (g_cad_pc->size()<=0)
        {
            ROS_ERROR("Error loading CAD cloud from %s",TARGET_CLOUD_TOPIC.c_str());
            new_scan_pc = false;
        }
    }
}

void scanCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!new_scan_pc)
    {
        ROS_INFO("Get SCAN cloud");
        // Save cloud
        pcl::fromROSMsg(*msg, *g_scan_pc);
        new_scan_pc = true;

        if (g_scan_pc->size()<=0)
        {
            ROS_ERROR("Error loading SCAN cloud from %s",SOURCE_CLOUD_TOPIC.c_str());
            new_scan_pc = false;
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle nh;
    ros::Rate r(FREQ);

    ROS_INFO("%s",ros::this_node::getName().c_str());

    // PARAMETERS
    std::string pointcloud_folder_path;
    if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))   
    {
        ROS_ERROR("input_cloud: No pointcloud folder path on Param Server");
        return 0;
    }

    PointCloudRGB::Ptr cad_pc_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr scan_pc_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr scan_pc_aligned(new PointCloudRGB);

    sensor_msgs::PointCloud2 cad_cloud_msg, scan_cloud_msg;
    std::vector<sensor_msgs::PointCloud2> cluster_msg_array;
    std_msgs::Int16 n_fods_msg;
    
    ros::Subscriber cad_sub = nh.subscribe(TARGET_CLOUD_TOPIC, 1, cadCb);
    ros::Subscriber scan_sub = nh.subscribe(SOURCE_CLOUD_TOPIC, 1, scanCb);

    ros::Publisher npub = nh.advertise<std_msgs::Int16>("/num_of_fods", 1);
    ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("/cad/cloud_filtered", 1);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan/cloud_aligned", 1);

    Eigen::Matrix4f final_transform; 

    while(ros::ok())
    {
        if (new_cad_pc && new_scan_pc)
        {
            ROS_INFO("Process started");

            // Filter clouds
            Filter cad_cloud_filter(0.05);
            Filter scan_cloud_filter(0.05, 10, 0.8);
            cad_cloud_filter.run(g_cad_pc, cad_pc_filtered);
            scan_cloud_filter.run(g_scan_pc, scan_pc_filtered);
            
            // Get initial alignment
            InitialAlignment initial_alignment(cad_pc_filtered, scan_pc_filtered);
            initial_alignment.run();
            Utils::printTransform(initial_alignment.getRigidTransform());
            initial_alignment.getAlignedCloud(scan_pc_aligned);

            // Get fine alignment
            GICPAlignment gicp_alignment(cad_pc_filtered, scan_pc_aligned);
            gicp_alignment.run();
            Utils::printTransform(gicp_alignment.getFineTransform());
            gicp_alignment.getAlignedCloud(scan_pc_aligned);
            ROS_INFO("Updated transform:");
            final_transform = gicp_alignment.getFineTransform() * initial_alignment.getRigidTransform();
            Utils::printTransform(final_transform);

            // Service para iterar gicp
            ros::ServiceServer gicp_service = nh.advertiseService("iterate_gicp", gicpCb);
            ros::ServiceServer undo_service = nh.advertiseService("undo_iteration", undoCb);
            ros::ServiceServer fods_service = nh.advertiseService("get_fods", fodsCb);

            // Convert to ROS data type
            Utils::cloudToROSMsg(cad_pc_filtered, cad_cloud_msg);
            Utils::cloudToROSMsg(scan_pc_aligned, scan_cloud_msg); 

            ROS_INFO("input_cloud: Publishing clouds on topics: \n\t\t\t\t/cad/cloud_filtered \n\t\t\t\t/scan/cloud_aligned");

            bool publish_fods = false;
            int num_of_fods=0;
            
            while(new_cad_pc && new_scan_pc && ros::ok())
            {
                // Publish the data
                cad_pub.publish(cad_cloud_msg);
                scan_pub.publish(scan_cloud_msg);

                if(iterate)
                {
                    gicp_alignment.iterate();
                    gicp_alignment.getAlignedCloudROSMsg(scan_cloud_msg);
                    ROS_INFO("Updated transform:");
                    final_transform = gicp_alignment.getFineTransform() * initial_alignment.getRigidTransform();
                    Utils::printTransform(final_transform);
                    iterate = false;
                }

                if (undo_last_iteration)
                {
                    gicp_alignment.undo();
                    gicp_alignment.getAlignedCloudROSMsg(scan_cloud_msg);
                    undo_last_iteration = false;
                }

                if (get_fods)
                {
                    gicp_alignment.getAlignedCloud(scan_pc_aligned);
                    BooleanDifference boolean_difference(scan_pc_aligned);
                    boolean_difference.substract(cad_pc_filtered);
                    if (!boolean_difference.substract_error)
                    {
                        PointCloudRGB::Ptr scan_pc_substracted(new PointCloudRGB);
                        boolean_difference.getResultCloud(scan_pc_substracted);
                        Viewer::visualizePointCloud<pcl::PointXYZRGB>(scan_pc_substracted);
                    
                        FODDetector fod_detector(boolean_difference.getVoxelResolution());
                        std::vector<pcl::PointIndices> cluster_indices; //This is a vector of cluster
                        fod_detector.clusterPossibleFODs(scan_pc_substracted, cluster_indices);

                        // iterate through cluster_indices to create a pc for each cluster
                        num_of_fods = fod_detector.clusterIndicesToROSMsg(cluster_indices, scan_pc_substracted, cluster_msg_array);
                        n_fods_msg.data = num_of_fods;
                        ROS_INFO("Detected %d objects",num_of_fods);
                
                        publish_fods = true;
                    }
                    get_fods = false;
                }

                if (publish_fods)
                {
                    std::vector<ros::Publisher> pub_array;
                    // publish each fod in a separated topic
                    for(int i=0; i<num_of_fods; i++) // temporary solution
                    {
                        std::string topic_name = "/fod" + std::to_string(i);
                        ROS_INFO("Publishing on topic %s", topic_name.c_str());
                        ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
                        pub_array.push_back(pub);
                    }
                    for(int i=0; i<num_of_fods; i++) // temporary solution
                    {
                        ROS_INFO("Point cloud size: %dx%d", cluster_msg_array[i].width, cluster_msg_array[i].height);
                        pub_array[i].publish(cluster_msg_array[i]);
                    }
                    npub.publish(n_fods_msg);

                    new_cad_pc  = false; // TODO ESTO NO VA AQUI DEBE HABER ALGO Q LO ACTIVE
                    new_scan_pc = false;
                }

                ros::spinOnce();
                r.sleep();
            }
        }
    
        ros::spinOnce();
    }

    // ros::spin();

    return 0;
}