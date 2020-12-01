// Bring in my package's API, which is what I'm testing
#include "Utils.h"
// Bring in gtest
#include <gtest/gtest.h>
#include <exception>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class TestUtils : public ::testing::Test {

    protected:

        TestUtils(){}

        void cubeXYZ(PointCloudXYZ::Ptr cloud, float dim, float step){
            pcl::PointXYZ p_rgb;
            for (float i=0; i<dim; i+=step) {
                for (float j=0; j<dim; j+=step){
                    for (float k=0; k<dim; k+=step){
                        p_rgb.x = i; 
                        p_rgb.y = j; 
                        p_rgb.z = k;
                        cloud->push_back(p_rgb);
                    }
                }
            }
        }

        void cubeRGB(PointCloudRGB::Ptr cloud, float dim, float step){
            pcl::PointXYZRGB p_rgb;
            for (float i=0; i<dim; i+=step) {
                for (float j=0; j<dim; j+=step){
                    for (float k=0; k<dim; k+=step){
                        p_rgb.x = i; 
                        p_rgb.y = j; 
                        p_rgb.z = k; 
                        p_rgb.r = 255; 
                        p_rgb.g = 255; 
                        p_rgb.b = 255; 
                        cloud->push_back(p_rgb);
                    }
                }
            }
        }
};

TEST_F(TestUtils, testValidCloud)
{
    PointCloudXYZ::Ptr cloudXYZ {new pcl::PointCloud<pcl::PointXYZ>};
    PointCloudRGB::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    
    EXPECT_FALSE(Utils::isValidCloud(cloudXYZ));
    EXPECT_FALSE(Utils::isValidCloud(cloudRGB));
    
    cubeXYZ(cloudXYZ, 3, 0.1);
    cubeRGB(cloudRGB, 3, 0.1);
    
    EXPECT_TRUE(Utils::isValidCloud(cloudXYZ));
    EXPECT_TRUE(Utils::isValidCloud(cloudRGB));
}

TEST_F(TestUtils, testCloudResolution)
{
    PointCloudXYZ::Ptr cloudXYZ {new pcl::PointCloud<pcl::PointXYZ>};
    PointCloudRGB::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    
    double res = 0.1;
    cubeXYZ(cloudXYZ, 3, res);
    cubeRGB(cloudRGB, 3, res);

    double res_pc = Utils::computeCloudResolution(cloudXYZ);
    EXPECT_TRUE((res_pc >= res-0.05) && (res_pc <= res+0.05));

    res = Utils::computeCloudResolution(cloudRGB);
    EXPECT_TRUE((res_pc >= res-0.05) && (res_pc <= res+0.05));
}

TEST_F(TestUtils, testCloudConversions)
{
    PointCloudXYZ::Ptr cloudXYZ {new pcl::PointCloud<pcl::PointXYZ>};
    PointCloudRGB::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};

    cubeXYZ(cloudXYZ, 3, 0.1);
    Utils::cloudToXYZRGB(cloudXYZ, cloudRGB, 255, 255, 255);
}

TEST_F(TestUtils, testCloudToROSMsg)
{
    ros::Time::init(); // because message stamp need ros::Time::now()

    PointCloudRGB::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    cubeRGB(cloudRGB, 3, 0.1);

    sensor_msgs::PointCloud2 cloudMsg;
    Utils::cloudToROSMsg(cloudRGB, cloudMsg);
}

TEST_F(TestUtils, testCloudTranslate)
{
    PointCloudRGB::Ptr cloudRGB_orig {new pcl::PointCloud<pcl::PointXYZRGB>};
    PointCloudRGB::Ptr cloudRGB_tras {new pcl::PointCloud<pcl::PointXYZRGB>};

    cubeRGB(cloudRGB_orig, 3, 0.1);
    Utils::translateCloud(cloudRGB_orig, cloudRGB_tras, 5, 0, 0);
    std::cout << cloudRGB_tras->points[0].x << std::endl;
    ASSERT_EQ(cloudRGB_tras->points[0].x, 5);

    Utils::translateCloud(cloudRGB_tras, cloudRGB_tras, -5, 0, 0);
    std::cout << cloudRGB_tras->points[0].x << std::endl;
    ASSERT_EQ(cloudRGB_tras->points[0].x, 0);
}

// TEST_F(TestUtils, testCloudRotate)
// {
//     // create RGB pointcloud
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in {new pcl::PointCloud<pcl::PointXYZRGB>};
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out {new pcl::PointCloud<pcl::PointXYZRGB>};
//     cubePointCloud(cloud_in, 5, 1);

//     Utils::rotateCloud(cloud_in, cloud_out, 5, 0, 0);
//     ASSERT_NE(*cloud_orig, *cloud_in);
// }


// getNormals
// printTransform complicado probarla en este test ? (se usa en initial_alignment.getRigidTransform)
// indicesFilter complicado probarla en este test ? (se usa en align.cpp)


// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}