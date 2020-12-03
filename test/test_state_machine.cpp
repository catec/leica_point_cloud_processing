#include <ros/ros.h>
#include <gtest/gtest.h>
#include <Utils.h>
#include <LeicaStateMachine.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class TestStateMachine : public ::testing::Test 
{ 
protected:

    PointCloudRGB::Ptr cloudRGB {new PointCloudRGB};

    TestStateMachine(){
        cubePointCloud(cloudRGB, 1, 10000);
    }

    void cubePointCloud(PointCloudRGB::Ptr cloud, float dim, int nsamples){
        pcl::PointXYZRGB p_rgb;
        for(int i=0; i<nsamples; i++)
        {
            p_rgb.x = dim * (double)std::rand() / (double)RAND_MAX;
            p_rgb.y = dim * (double)std::rand() / (double)RAND_MAX;
            p_rgb.z = dim * (double)std::rand() / (double)RAND_MAX;
            p_rgb.r = 255;
            p_rgb.g = 255;
            p_rgb.b = 255;
            cloud->push_back(p_rgb);
        } 
    }
};

// Test state machine
TEST_F(TestStateMachine, testEvents)
{
    ros::NodeHandle nh;
    PointCloudRGB::Ptr source_cloud = cloudRGB;
    PointCloudRGB::Ptr target_cloud = cloudRGB;

    if (Utils::isValidCloud(source_cloud) && Utils::isValidCloud(target_cloud))
    {
        std::shared_ptr<StateMachine> FSM = std::make_shared<StateMachine>(source_cloud, target_cloud, nh);
        FSM->initiate();
        try
        {
            FSM->state_cast<const IdleState &>();
        }
        catch(std::exception& e)
        {
            ADD_FAILURE() << "Not in IdleState";
        }

        FSM->process_event(StartEvent());
        try
        {
            FSM->state_cast<const PublishState &>();
        }
        catch(std::exception& e)
        {
            ADD_FAILURE() << "Transit to PublishState failed";
        }
    }
}

int main(int argc,char **argv)
{
  ros::init(argc, argv, "test_state_machine");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}