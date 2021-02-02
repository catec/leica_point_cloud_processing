/**
 * @file test_cad_to_pointcloud.cpp
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

#include "CADToPointCloud.h"
#include "leica_scanstation_utils/LeicaUtils.h"
#include <gtest/gtest.h>


// Test  CADToPointCloud constructor should return success
TEST(TestCADToPointCloud, testPLY)
{
    std::string cad_folder_path = ros::package::getPath("leica_point_cloud_processing");
    std::string file_name = cad_folder_path + "/test/cube.ply";

    int sample_points = 10000;
    
    PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};

    CADToPointCloud cad2pc(file_name, sample_points);
    cad2pc.convertCloud(cubeRGB);

    ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
    ASSERT_GT(cubeRGB->size(),1);
}

// Test  CADToPointCloud constructor should return success
TEST(TestCADToPointCloud, testOBJ)
{
    std::string cad_folder_path = ros::package::getPath("leica_point_cloud_processing");
    std::string file_name = cad_folder_path + "/test/cube.ply";

    int sample_points = 10000;
    
    PointCloudRGB::Ptr cubeRGB {new PointCloudRGB};

    CADToPointCloud cad2pc(file_name, sample_points);
    cad2pc.convertCloud(cubeRGB);

    ASSERT_TRUE(Utils::isValidCloud(cubeRGB));
    ASSERT_GT(cubeRGB->size(),1);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}