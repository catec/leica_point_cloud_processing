/**
 * @file test_initial_alignment.cpp
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

#include "InitialAlignment.h"
#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include <Viewer.h>
#include <gtest/gtest.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class TestInitialAlignment : public ::testing::Test
{ 
protected:

    PointCloudRGB::Ptr cloudRGB {new PointCloudRGB};
    PointCloudRGB::Ptr sourceRGB {new PointCloudRGB};
    PointCloudRGB::Ptr targetRGB {new PointCloudRGB};

    TestInitialAlignment()
    {
        cubePointCloud(sourceRGB);
        Utils::translateCloud(sourceRGB, targetRGB, 0.05, 0.05, 0.05);
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud)
    {
        std::string pkg_path = ros::package::getPath("leica_point_cloud_processing");
        std::string f = pkg_path + "/test/cube.ply";

        int sample_points = 20000;
        
        CADToPointCloud cad2pc(f, sample_points);
        cad2pc.convertCloud(cloud);
    }
};

TEST_F(TestInitialAlignment, testApplyTF)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);

    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    initial_alignment.run(); // run method works well
    
    initial_alignment.applyTFtoCloud(sourceRGB);

    double tolerance = 1e-3;
    // check first point for translation result is conincident with first point in target
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
}

TEST_F(TestInitialAlignment, testTF)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);

    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    ASSERT_EQ(tf, Eigen::Matrix4f::Identity());  // constructor works well
    
    initial_alignment.run(); // run method works well
    
    tf = initial_alignment.getRigidTransform();

    double translation = 0.05;
    double tolerance = 1e-3;
    // check translation result is translation value applied to target
    EXPECT_TRUE(abs(tf.col(3)[0]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[1]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[2]-translation) <= tolerance);
}

TEST_F(TestInitialAlignment, testHarrisAlg)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);
    initial_alignment.setMethod(AlignmentMethod::HARRIS);
    initial_alignment.run(); // run method works well
    
    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    double translation = 0.05;
    double tolerance = 1e-3;
    // check translation result is translation value applied to target
    EXPECT_TRUE(abs(tf.col(3)[0]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[1]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[2]-translation) <= tolerance);
}

TEST_F(TestInitialAlignment, testMultiscaleAlg)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);
    initial_alignment.setMethod(AlignmentMethod::MULTISCALE);
    initial_alignment.run(); // run method works well
    
    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    double translation = 0.05;
    double tolerance = 1e-3;
    // check translation result is translation value applied to target
    EXPECT_TRUE(abs(tf.col(3)[0]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[1]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[2]-translation) <= tolerance);
}

TEST_F(TestInitialAlignment, testNormalsAlg)
{
    InitialAlignment initial_alignment(targetRGB, sourceRGB);
    initial_alignment.setMethod(AlignmentMethod::NORMALS);
    initial_alignment.run(); // run method works well
    
    Eigen::Matrix4f tf = initial_alignment.getRigidTransform();
    double translation = 0.05;
    double tolerance = 1e-3;
    // check translation result is translation value applied to target
    EXPECT_TRUE(abs(tf.col(3)[0]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[1]-translation) <= tolerance);
    EXPECT_TRUE(abs(tf.col(3)[2]-translation) <= tolerance);

    initial_alignment.applyTFtoCloud(sourceRGB);
    // check first point for translation result is conincident with first point in target
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
    EXPECT_TRUE(abs(sourceRGB->points[0].x-targetRGB->points[0].x) <= tolerance);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}