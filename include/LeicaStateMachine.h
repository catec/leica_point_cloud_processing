#pragma once
#ifndef _LEICA_STATE_MACHINE_H
#define _LEICA_STATE_MACHINE_H

// #include <iostream>
#include <boost/statechart/event.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>

#include <Utils.h>
#include <std_msgs/Int16.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// Events
class ValidEvent : public boost::statechart::event<ValidEvent> 
{
public:
    PointCloudRGB::Ptr source_cloud_;
    PointCloudRGB::Ptr target_cloud_;

    ValidEvent(PointCloudRGB::Ptr source_cloud) 
        : source_cloud_(source_cloud) {}

    ValidEvent(PointCloudRGB::Ptr source_cloud, PointCloudRGB::Ptr target_cloud)
        : source_cloud_(source_cloud), target_cloud_(target_cloud){}
};

class ValidFODEvent : public boost::statechart::event<ValidFODEvent>
{
public:
    int n_fod_;
    std::vector<PointCloudRGB::Ptr> fods_cloud_array_;
    
    ValidFODEvent(int n_fods, std::vector<PointCloudRGB::Ptr> fods_cloud_array)
        : n_fod_(n_fods), fods_cloud_array_(fods_cloud_array){}
};

class NoValidEvent : public boost::statechart::event<NoValidEvent> {};
class StartEvent : public boost::statechart::event<StartEvent> {};
class PublishEvent : public boost::statechart::event<PublishEvent> {};
class StopPublishEvent : public boost::statechart::event<StopPublishEvent> {};

// States
class IdleState;
class FilterState;
class AlignState;
class GICPState;
class FODDetectionState;
class ErrorState;
class PublishState;
class CallbackState;

// State Machine
class StateMachine : public boost::statechart::state_machine< StateMachine, IdleState > 
{
public:

    StateMachine(PointCloudRGB::Ptr source_cloud, PointCloudRGB::Ptr target_cloud, ros::NodeHandle nh);

    PointCloudRGB::Ptr source_cloud_;
    PointCloudRGB::Ptr target_cloud_;
    PointCloudRGB::Ptr source_cloud_filtered_;
    PointCloudRGB::Ptr target_cloud_filtered_;
    Eigen::Matrix4f cloud_transform_;

    int n_fod_;
    std::vector<PointCloudRGB::Ptr> fods_cloud_array_;

    ros::NodeHandle nh_;
    ros::Publisher source_pub_;
    ros::Publisher target_pub_;
    ros::Publisher n_fods_pub_;   
    ros::Publisher fods_pub_; 

    void updateCloud(const ValidEvent& cloud)
    {
        ROS_INFO("Updating source cloud");
        pcl::copyPointCloud(*cloud.source_cloud_, *source_cloud_);
    }

    void updateClouds(const ValidEvent& clouds)
    {
        ROS_INFO("Updating clouds");
        pcl::copyPointCloud(*clouds.source_cloud_, *source_cloud_);
        pcl::copyPointCloud(*clouds.target_cloud_, *target_cloud_);
    }

    void storeFODInfo(const ValidFODEvent& fod_info)
    {
        ROS_INFO("Storing FODs info");
        n_fod_ = fod_info.n_fod_;
        fods_cloud_array_ = fod_info.fods_cloud_array_;
    }
    
    PointCloudRGB::Ptr getTargetCloud()
    {
        return target_cloud_;
    }
    PointCloudRGB::Ptr getSourceCloud()
    {
        return source_cloud_;
    }

    PointCloudRGB::Ptr getSourceCloudFiltered()
    {
        return source_cloud_filtered_;
    }
    PointCloudRGB::Ptr getTargetCloudFiltered()
    {
        return target_cloud_filtered_;
    }

    void setSourceCloudFiltered(PointCloudRGB::Ptr cloud_ptr)
    {
        source_cloud_filtered_ = cloud_ptr;
    }
    void setTargetCloudFiltered(PointCloudRGB::Ptr cloud_ptr)
    {
        target_cloud_filtered_ = cloud_ptr;
    }
    
    Eigen::Matrix4f getCloudTransform()
    {
        return cloud_transform_;
    }

    void setCloudTransform(Eigen::Matrix4f tf)
    {
        cloud_transform_ = tf;
    }

    int getNFODs()
    {
        return n_fod_;
    }

    std::vector<PointCloudRGB::Ptr> getFODCloudArray()
    {
        return fods_cloud_array_;
    }
};

// States
class IdleState : public boost::statechart::simple_state< IdleState, StateMachine >
{
public:
    IdleState()
    { ROS_INFO("Into IdleState."); }

    typedef boost::statechart::custom_reaction<StartEvent> reactions;

    boost::statechart::result react(const StartEvent&)
    {
        ROS_INFO("Transit to FilterState");
        return transit< FilterState >();
    }
};

class FilterState : public boost::statechart::state< FilterState, StateMachine > 
{
public:
    FilterState(my_context ctx);
    
    typedef boost::mpl::list< 
        boost::statechart::custom_reaction<ValidEvent>,
        boost::statechart::custom_reaction<NoValidEvent> 
        > reactions;

    boost::statechart::result react(const NoValidEvent&)
    {
        ROS_ERROR("Transit to Error");
        return transit< ErrorState >();
    }

    boost::statechart::result react(const ValidEvent& clouds)
    {
        ROS_INFO("Transit to AlignState");
        return transit< AlignState >(&StateMachine::updateClouds, clouds);
    }
};

class AlignState : public boost::statechart::state< AlignState, StateMachine >
{
public:
    AlignState(my_context ctx);

    typedef boost::mpl::list< 
        boost::statechart::custom_reaction<ValidEvent>,
        boost::statechart::custom_reaction<NoValidEvent> 
        > reactions;

    boost::statechart::result react(const NoValidEvent&)
    {
        ROS_ERROR("Transit to Error");
        return transit< ErrorState >();
    }

    boost::statechart::result react(const ValidEvent& cloud)
    {
        ROS_INFO("Transit to GICPState");
        return transit< GICPState >(&StateMachine::updateCloud, cloud);
    }
};

class GICPState : public boost::statechart::state< GICPState, StateMachine >
{
public:
    GICPState(my_context ctx);
    
    typedef boost::mpl::list< 
        boost::statechart::custom_reaction<ValidEvent>, 
        boost::statechart::custom_reaction<NoValidEvent>
        > reactions;

    boost::statechart::result react(const NoValidEvent&)
    {
        ROS_ERROR("Transit to Error");
        return transit< ErrorState >();
    }

    boost::statechart::result react(const ValidEvent& cloud)
    {
        ROS_INFO("Transit to FODDetectionState");
        return transit< FODDetectionState >(&StateMachine::updateCloud, cloud);
    }
};

class FODDetectionState : public boost::statechart::state< FODDetectionState, StateMachine >
{
public:
    FODDetectionState(my_context ctx);
    
    typedef boost::mpl::list< 
        boost::statechart::custom_reaction<ValidFODEvent>, 
        boost::statechart::custom_reaction<NoValidEvent>
        > reactions;

    boost::statechart::result react(const NoValidEvent&)
    {
        ROS_ERROR("Transit to Error");
        return transit< ErrorState >();
    }

    boost::statechart::result react(const ValidFODEvent& cloud)
    {
        ROS_INFO("Transit to PublishState");
        return transit< PublishState >(&StateMachine::storeFODInfo, cloud);
    }
};

class ErrorState : public boost::statechart::simple_state< ErrorState, StateMachine >
{
public:
    ErrorState() { ROS_INFO("Into ErrorState"); };
};

class PublishState : public boost::statechart::state< PublishState, StateMachine >
{
public:
    PublishState(my_context ctx);

    typedef boost::mpl::list< 
        boost::statechart::custom_reaction<PublishEvent>, 
        boost::statechart::custom_reaction<StopPublishEvent>
        > reactions;

    boost::statechart::result react(const PublishEvent&)
    {
        ROS_INFO("Loop on publish");
        return transit< PublishState >();
    }

    boost::statechart::result react(const StopPublishEvent&)
    {
        ROS_INFO("Stop publish");
    }
};

#endif