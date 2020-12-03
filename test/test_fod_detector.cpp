#include "FODDetector.h"
#include "Utils.h"
#include <gtest/gtest.h>
#include <exception>
#include <Viewer.h>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class TestFODDetector : public ::testing::Test
{ 
    protected:

        TestFODDetector(){}

        void cubesPointCloud(PointCloudRGB::Ptr cloud, float pos, float dim, float step){
            pcl::PointXYZRGB p_rgb;
            for (float i=pos; i<pos+dim; i+=step) {
                for (float j=pos; j<pos+dim; j+=step){
                    for (float k=pos; k<pos+dim; k+=step){
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

// Test TestFODDetector should return 3 cluster indices
TEST_F(TestFODDetector, testFODClustering)
{
    PointCloudRGB::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    
    cubesPointCloud(cloudRGB,  0, 3, 0.1);
    cubesPointCloud(cloudRGB, 10, 3, 0.1);
    cubesPointCloud(cloudRGB, 20, 3, 0.1);

    // Viewer v;
    // v.addPCToViewer<pcl::PointXYZRGB>(cloudRGB, "cloudRGB");

    int min_fod_points = 3;
    double voxelize_factor = 3;
    double th = 4e-3 * voxelize_factor;

    FODDetector fod_detector(cloudRGB, th*10, min_fod_points);
    fod_detector.clusterPossibleFODs();

    std::vector<PointCloudRGB::Ptr> fods_cloud_array;
    int num_of_fods = fod_detector.fodIndicesToPointCloud(fods_cloud_array);

    // std::cout << num_of_fods << std::endl;
    ASSERT_EQ(num_of_fods,3);
    ASSERT_EQ(fods_cloud_array.size(),3);
}

// Test TestFODDetector should return error with empty cloud
TEST_F(TestFODDetector, testEmptyCloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr empty_cloud {new pcl::PointCloud<pcl::PointXYZRGB>};
    
    int min_fod_points = 3;
    double voxelize_factor = 3;
    double th = 4e-3 * voxelize_factor;

    FODDetector fod_detector(empty_cloud, th*10, min_fod_points);
    fod_detector.clusterPossibleFODs();

    std::vector<PointCloudRGB::Ptr> fods_cloud_array;
    int num_of_fods = fod_detector.fodIndicesToPointCloud(fods_cloud_array);

    // std::cout << num_of_fods << std::endl;
    ASSERT_EQ(num_of_fods, 0);
    ASSERT_EQ(fods_cloud_array.size(), 0);
}

// // Test testFODROSmsg should return success
TEST_F(TestFODDetector, testFODmsg)
{
    ros::Time::init();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB {new pcl::PointCloud<pcl::PointXYZRGB>};
    
    cubesPointCloud(cloudRGB,  0, 3, 0.1);
    cubesPointCloud(cloudRGB, 10, 3, 0.1);
    cubesPointCloud(cloudRGB, 20, 3, 0.1);

    int min_fod_points = 3;
    double voxelize_factor = 3;
    double th = 4e-3 * voxelize_factor;

    FODDetector fod_detector(cloudRGB, th*10, min_fod_points);
    fod_detector.clusterPossibleFODs();

    std::vector<sensor_msgs::PointCloud2> cluster_msg_array;
    int num_of_fods = fod_detector.fodIndicesToROSMsg(cluster_msg_array);

    // std::cout << num_of_fods << std::endl;
    ASSERT_EQ(num_of_fods, 3);
    ASSERT_EQ(cluster_msg_array.size(), 3);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}