/**
 * @file node.cpp
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

#include <LeicaStateMachine.h>
#include "leica_scanstation_utils/LeicaUtils.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

const float FREQ = 0.1;   // Hz

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
**/

int main(int argc, char** argv)
{
    ros::init(argc, argv, "statemachine");
    ros::NodeHandle nh;
    ros::Rate r(FREQ);

    PointCloudRGB::Ptr source_cloud(new PointCloudRGB);
    PointCloudRGB::Ptr target_cloud(new PointCloudRGB);

    sensor_msgs::PointCloud2ConstPtr cloud_msg; 
    cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(SOURCE_CLOUD_TOPIC);
    pcl::fromROSMsg(*cloud_msg, *source_cloud);
    ROS_INFO("Received cloud from topic: %s", SOURCE_CLOUD_TOPIC.c_str());
    
    cloud_msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(TARGET_CLOUD_TOPIC);
    pcl::fromROSMsg(*cloud_msg, *target_cloud);
    ROS_INFO("Received cloud from topic: %s", TARGET_CLOUD_TOPIC.c_str());

    if (Utils::isValidCloud(source_cloud) && Utils::isValidCloud(target_cloud))
    {
        std::shared_ptr<StateMachine> FSM = std::make_shared<StateMachine>(source_cloud, target_cloud, nh);
        FSM->initiate();
        FSM->process_event(StartEvent());
        
        while(ros::ok())
        {
            FSM->process_event(PublishEvent());
            ros::spinOnce();
            r.sleep();
        }
    }

    ROS_ERROR("NO VALID MSGS");

    ros::spin();

    return 0;
}