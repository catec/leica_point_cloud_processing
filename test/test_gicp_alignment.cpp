/**
 * @file test_gicp_alignment.cpp
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

#include "GICPAlignment.h"
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include "Utils.h"
#include <Viewer.h>
#include <gtest/gtest.h>

class TestGICPAlignment : public ::testing::Test
{ 
protected:

    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};
    
    TestGICPAlignment()
    {
        cubePointCloud(sourceRGB);
        Utils::rotateCloud(sourceRGB, targetRGB, 0, 0, 0.175); // radians rotation in yaw
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud)
    {
        std::string pkg_path = ros::package::getPath("leica_point_cloud_processing");
        std::string f = pkg_path + "/test/cube.ply";

        int sample_points = 5000;
        
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cloud);
    }
};

TEST_F(TestGICPAlignment, testApplyTF)
{
    GICPAlignment gicp_alignment(targetRGB, sourceRGB, false);

    Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        
        gicp_alignment.run();

        gicp_alignment.applyTFtoCloud(sourceRGB);

        double tolerance = 1e-3;
        // check first point for translation result is conincident with first point in target
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
        EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }
}

TEST_F(TestGICPAlignment, testRun)
{
    GICPAlignment gicp_alignment(targetRGB, sourceRGB, false);
    
    Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well

    gicp_alignment.setMaxIterations(100);
    gicp_alignment.setMaxCorrespondenceDistance(5);
    gicp_alignment.setRANSACOutlierTh(5e-2);
    gicp_alignment.setTfEpsilon(5e-4);

    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        
        gicp_alignment.run();

        PointCloudRGB::Ptr aligned_cloud {new PointCloudRGB};
        gicp_alignment.getAlignedCloud(aligned_cloud);

        EXPECT_TRUE(gicp_alignment.transform_exists_);
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }
}

TEST_F(TestGICPAlignment, testRunWithCov)
{
    GICPAlignment gicp_alignment(targetRGB, sourceRGB, true);
    
    Eigen::Matrix4f tf = gicp_alignment.getFineTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well

    try
    {
        ros::Time::init(); // because need of ros::Time::now()
        
        gicp_alignment.run();

        PointCloudRGB::Ptr aligned_cloud {new PointCloudRGB};
        gicp_alignment.getAlignedCloud(aligned_cloud);

        EXPECT_TRUE(gicp_alignment.transform_exists_);

        gicp_alignment.iterate();
        EXPECT_TRUE(gicp_alignment.transform_exists_);
    }
    catch(std::exception& e)
    {
        ADD_FAILURE()<< e.what();
    }
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}