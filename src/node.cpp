#include <Utils.h>

#include <CADToPointCloud.h>
#include <Filter.h>
#include <InitialAlignment.h>
#include <GICPAlignment.h>
#include <BooleanDifference.h>
#include <FODDetector.h>
#include <Viewer.h>

#include "std_srvs/Trigger.h"
#include "std_msgs/Int16.h"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

std::string TARGET_CLOUD_TOPIC = "/cad/cloud";
std::string SOURCE_CLOUD_TOPIC = "/scan/cloud";
int FREQ = 5; // Hz

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/
PointCloudRGB::Ptr g_cad_cloud(new PointCloudRGB);
PointCloudRGB::Ptr g_scan_cloud(new PointCloudRGB);

// Flags to control program flow
bool new_cad_cloud = false;
bool new_scan_cloud = false;
bool iterate = false;
bool undo_last_iteration = false;
bool get_fods = false;
bool stop_alignment = false;


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

bool stopCb(std_srvs::Trigger::Request  &req,
            std_srvs::Trigger::Response &res)
{
    stop_alignment = true; 

    res.success = true;
    res.message = "stop";
}

void cadCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!new_cad_cloud)
    {
        ROS_INFO("Get CAD cloud");
        // Save cloud
        pcl::fromROSMsg(*msg, *g_cad_cloud);

        if (Utils::isValidCloud(g_cad_cloud))
            new_cad_cloud = true;
        else
            ROS_ERROR("Error loading CAD cloud from %s",TARGET_CLOUD_TOPIC.c_str());
    }
}

void scanCb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    if (!new_scan_cloud)
    {
        ROS_INFO("Get SCAN cloud");
        // Save cloud
        pcl::fromROSMsg(*msg, *g_scan_cloud);
        
        if (Utils::isValidCloud(g_scan_cloud))
            new_scan_cloud = true;
        else
            ROS_ERROR("Error loading SCAN cloud from %s",SOURCE_CLOUD_TOPIC.c_str());
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_align");
    ros::NodeHandle nh;
    ros::Rate r(FREQ);

    ROS_INFO("%s",ros::this_node::getName().c_str());

    // PARAMETERS
    std::string pointcloud_folder_path;
    if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))   
    {
        ROS_ERROR("No pointcloud folder path on Param Server");
        return 0;
    }

    PointCloudRGB::Ptr cad_cloud_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr scan_cloud_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr scan_cloud_aligned(new PointCloudRGB);

    sensor_msgs::PointCloud2 cad_cloud_msg, scan_cloud_msg;
    std::vector<sensor_msgs::PointCloud2> cluster_msg_array;
    std_msgs::Int16 n_fods_msg;
    
    ros::Subscriber cad_sub = nh.subscribe(TARGET_CLOUD_TOPIC, 1, cadCb);
    ros::Subscriber scan_sub = nh.subscribe(SOURCE_CLOUD_TOPIC, 1, scanCb);

    ros::Publisher npub = nh.advertise<std_msgs::Int16>("/num_of_fods", 1);
    ros::Publisher cad_pub = nh.advertise<sensor_msgs::PointCloud2>("/cad/cloud_filtered", 1);
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::PointCloud2>("/scan/cloud_aligned", 1);
    std::vector<ros::Publisher> pub_array; // one publisher in array for each FOD detected

    Eigen::Matrix4f final_transform; 

    while(ros::ok())
    {
        if (new_cad_cloud && new_scan_cloud && ros::ok())
        {
            ROS_INFO("Process started");

            // Set correct leaf_size
            double cad_res = Utils::computeCloudResolution(g_cad_cloud);
            double scan_res = Utils::computeCloudResolution(g_scan_cloud);
            double leaf_size = 15 * std::max(cad_res, scan_res); // 15 times higher works well in most cases

            // Filter parameters (todo: set dynamically and from launch)
            double part_size = 0.9;      // Larger side of the part
            double floor_height = 0.35;  // Maximum height to search for floor  
            Eigen::Vector3f part_center; // coordinates from LeicaScanstation frame
            part_center[0] = 0.05; // x
            part_center[1] = -1.5; // y
            part_center[2] = 0;    // z

            // Filter clouds
            Filter cad_cloud_filter(leaf_size);
            Filter scan_cloud_filter(part_center, leaf_size, part_size, floor_height);
            cad_cloud_filter.run(g_cad_cloud, cad_cloud_filtered);
            scan_cloud_filter.run(g_scan_cloud, scan_cloud_filtered);
            
            // Perform initial alignment
            InitialAlignment initial_alignment(cad_cloud_filtered, scan_cloud_filtered);
            initial_alignment.run();
            Utils::printTransform(initial_alignment.getRigidTransform());
            initial_alignment.getAlignedCloud(scan_cloud_aligned);
            
            if (!Utils::isValidCloud(scan_cloud_aligned)) return 0;

            // Perform fine alignment   
            GICPAlignment gicp_alignment(cad_cloud_filtered, scan_cloud_aligned);
            gicp_alignment.run();
            Utils::printTransform(gicp_alignment.getFineTransform());
            gicp_alignment.getAlignedCloud(scan_cloud_aligned);
            ROS_INFO("Updated transform:");
            final_transform = gicp_alignment.getFineTransform() * initial_alignment.getRigidTransform();
            Utils::printTransform(final_transform);

            // Services to communicate with node
            ros::ServiceServer gicp_service = nh.advertiseService("iterate_gicp", gicpCb);
            ros::ServiceServer undo_service = nh.advertiseService("undo_iteration", undoCb);
            ros::ServiceServer fods_service = nh.advertiseService("get_fods", fodsCb);
            ros::ServiceServer stop_service = nh.advertiseService("stop_alignment", stopCb);

            // Convert cad cloud and aligned cloud to ROS data type
            Utils::cloudToROSMsg(cad_cloud_filtered, cad_cloud_msg);
            Utils::cloudToROSMsg(scan_cloud_aligned, scan_cloud_msg); 

            ROS_INFO("Publishing clouds on topics: \n\t\t\t\t/cad/cloud_filtered \n\t\t\t\t/scan/cloud_aligned");

            bool publish_fods = false;
            int num_of_fods = 0;
            
            while(new_cad_cloud && new_scan_cloud && ros::ok())
            {
                // Publish clouds
                cad_pub.publish(cad_cloud_msg);
                scan_pub.publish(scan_cloud_msg);

                // Following functions will only apply if required by user (via ROS services)
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
                    gicp_alignment.getAlignedCloud(scan_cloud_aligned);
                    BooleanDifference boolean_difference(scan_cloud_aligned);
                    boolean_difference.substract(cad_cloud_filtered);
                    if (!boolean_difference.substract_error)
                    {
                        PointCloudRGB::Ptr scan_cloud_substracted(new PointCloudRGB);
                        boolean_difference.getResultCloud(scan_cloud_substracted);
                    
                        FODDetector fod_detector(boolean_difference.getVoxelResolution());
                        std::vector<pcl::PointIndices> cluster_indices; //This is a vector of cluster
                        fod_detector.clusterPossibleFODs(scan_cloud_substracted, cluster_indices);

                        // iterate through cluster_indices to create a pc for each cluster
                        num_of_fods = fod_detector.clusterIndicesToROSMsg(cluster_indices, scan_cloud_substracted, cluster_msg_array);
                        n_fods_msg.data = num_of_fods;
                        ROS_INFO("Detected %d objects",num_of_fods);

                        for(int i=0; i<num_of_fods; i++) 
                        {
                            std::string topic_name = "/fod" + std::to_string(i);
                            ROS_INFO("Publishing on topic %s", topic_name.c_str());
                            ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1);
                            pub_array.push_back(pub);
                        }
                        publish_fods = true;
                    }
                    get_fods = false;
                }

                if (publish_fods)
                {
                    // publish each fod in a separated topic
                    for(int i=0; i<num_of_fods; i++)
                    {
                        pub_array[i].publish(cluster_msg_array[i]);
                    }
                    npub.publish(n_fods_msg);
                }

                if (stop_alignment)
                {
                    publish_fods = false;
                    new_cad_cloud  = false;
                    new_scan_cloud = false;
                    stop_alignment = false;
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