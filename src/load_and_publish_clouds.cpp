/**
 * @file load_and_publish_clouds.cpp
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */ 

#include <Utils.h>

#include "leica_scanstation_msgs/PointCloudFile.h"
#include "leica_scanstation_utils/LeicaUtils.h"

#include <CADToPointCloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

const int FREQ = 1;  // Hz

class CloudLoader
{

public:
    CloudLoader(){};
    ~CloudLoader(){};

    sensor_msgs::PointCloud2 source_cloud_msg_;
    sensor_msgs::PointCloud2 target_cloud_msg_;

    bool serviceCb(leica_scanstation_msgs::PointCloudFile::Request& req,
                   leica_scanstation_msgs::PointCloudFile::Response& res)
    {
        ROS_INFO("Request to publish clouds");

        int r = loadCloud(req.source_cloud_file, source_cloud_msg_, 255, 0, 128);
        if(r==-1) return false;

        r = loadCloud(req.target_cloud_file, target_cloud_msg_, 0, 0, 255);
        if(r==-1) return false;

        ROS_INFO("Publishing clouds on topics: \n\t%s \n\t%s",  TARGET_CLOUD_TOPIC.c_str(), 
                                                                SOURCE_CLOUD_TOPIC.c_str());
        res.message = "Finished service to receive cloud";
        res.success = true;
        return true;
    }

    int loadCloud(const std::string& file_name,
                  sensor_msgs::PointCloud2& cloud_msg,
                  int R, int G, int B)
    {
        PointCloudRGB::Ptr cloud(new PointCloudRGB);

        std::string extension = boost::filesystem::extension(file_name);

        std::string f = LeicaUtils::getFilePath(file_name);
        if (file_name.empty() || extension.empty() || !boost::filesystem::exists(f))
        {
            ROS_ERROR("Couldn't read file %s", f.c_str());
            return -1;
        }

        if (extension==".pcd")
        {
            pcl::io::loadPCDFile<pcl::PointXYZRGB>(f, *cloud);
            if(!Utils::isValidCloud(cloud)) return -1;
            Utils::colorizeCloud(cloud, R, G, B);  // scanned cloud is pink
        }
        else if (extension==".ply" || extension==".obj")
        {
            int sample_points = 500000;
            CADToPointCloud cad2pc(f, sample_points); 
            cad2pc.convertCloud(cloud); 
            if(!Utils::isValidCloud(cloud)) return -1;
            Utils::colorizeCloud(cloud, 0, 0, 255);  // cad cloud is blue
        }  
        else
            ROS_ERROR("Couldn't read file %s", f.c_str());

        ROS_INFO("Loaded file: %s", f.c_str());
        Utils::cloudToROSMsg(cloud, cloud_msg);

        return 0;
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_publisher");
    ros::NodeHandle nh;

    std::string pointcloud_folder_path;
    if (!nh.getParam("/pointcloud_folder", pointcloud_folder_path))
    {
        pointcloud_folder_path = LeicaUtils::findPointcloudFolderPath();
    }

    ROS_INFO("Search for pointcloud in %s", pointcloud_folder_path.c_str());

    CloudLoader cl;

    ros::ServiceServer service = nh.advertiseService("publish_clouds", 
                                                     &CloudLoader::serviceCb, &cl);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("source/cloud", 1);
    ros::Publisher tar_pub = nh.advertise<sensor_msgs::PointCloud2>("target/cloud", 1);

    ROS_INFO("Service waiting for call to /publish_clouds");

    ros::Rate r(FREQ);
    while (ros::ok())
    {
        if (Utils::isValidCloudMsg(cl.source_cloud_msg_) && 
            Utils::isValidCloudMsg(cl.target_cloud_msg_))
        {
            pub.publish(cl.source_cloud_msg_);
            r.sleep();  
            tar_pub.publish(cl.target_cloud_msg_);
        }
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
