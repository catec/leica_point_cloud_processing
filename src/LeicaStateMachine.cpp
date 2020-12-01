#include <LeicaStateMachine.h>
#include <Filter.h>
#include <InitialAlignment.h>
#include <GICPAlignment.h>
#include <FODDetector.h>
#include <Viewer.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

StateMachine::StateMachine(PointCloudRGB::Ptr source_cloud, PointCloudRGB::Ptr target_cloud, ros::NodeHandle nh)
        : nh_(nh),
          source_cloud_(source_cloud), 
          target_cloud_(target_cloud),
          source_cloud_filtered_(new PointCloudRGB), 
          target_cloud_filtered_(new PointCloudRGB) 
{
    source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/scan/cloud_aligned", 1);
    target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cad/cloud_filtered", 1);
    fods_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("/scan/fods_detected", 1);
    n_fods_pub_ = nh_.advertise<std_msgs::Int16>("/scan/num_of_fods_detected", 1);
    cloud_transform_ = Eigen::Matrix4f::Identity();
}

FilterState::FilterState(my_context ctx) : my_base(ctx) 
{
    ROS_INFO("Into FilterState");

    PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloud();
    PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloud();

    // set params
    double part_max_size = 4; // Larger side of the part
    ros::param::get("/part_max_size", part_max_size);

    double floor_height = 0.35;   // Maximum height to search for floor
    ros::param::get("/floor_height", floor_height);

    const double INF = std::numeric_limits<double>::infinity();
    Eigen::Vector3f part_center(INF, INF, INF); // Coordinates for part center
    ros::param::get("/part_center_x", part_center[0]);
    ros::param::get("/part_center_y", part_center[1]);
    ros::param::get("/part_center_z", part_center[2]);

    double leaf_size_factor = 10;
    ros::param::get("/leaf_size_factor", leaf_size_factor);
    double cad_res = Utils::computeCloudResolution(cad_cloud);
    double scan_res = Utils::computeCloudResolution(scan_cloud);
    double leaf_size = leaf_size_factor * std::max(cad_res, scan_res);

    bool using_cad = false;    
    ros::param::get("/using_CAD", using_cad);

    // filter
    PointCloudRGB::Ptr scan_cloud_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr cad_cloud_filtered(new PointCloudRGB);
    PointCloudRGB::Ptr scan_cloud_downsampled(new PointCloudRGB);
    PointCloudRGB::Ptr cad_cloud_downsampled(new PointCloudRGB);

    Filter scan_cloud_filter(leaf_size, part_max_size, floor_height);
    if(part_center[0]!=INF && part_center[1]!=INF && part_center[2]!=INF)
        scan_cloud_filter.setCloudCenter(part_center);
    scan_cloud_filter.run(scan_cloud, scan_cloud_filtered);
    scan_cloud_filter.downsampleCloud(scan_cloud_filtered, scan_cloud_downsampled);

    if(!using_cad) // source and target are scans
    {
        scan_cloud_filter.run(cad_cloud, cad_cloud_filtered);
        scan_cloud_filter.downsampleCloud(cad_cloud_filtered, cad_cloud_downsampled);
    }
    else  // target is CAD
    {
        Filter cad_cloud_filter(leaf_size);
        cad_cloud_filter.run(cad_cloud, cad_cloud_filtered);
        cad_cloud_filter.downsampleCloud(cad_cloud, cad_cloud_downsampled);
    }
    
    // store on machine filtered and downsampled clouds
    context<StateMachine>().setTargetCloudFiltered(cad_cloud_filtered);
    context<StateMachine>().setSourceCloudFiltered(scan_cloud_filtered);

    if (Utils::isValidCloud(scan_cloud_downsampled) && Utils::isValidCloud(cad_cloud_downsampled))
        post_event(ValidEvent(scan_cloud_downsampled, cad_cloud_downsampled));
    else
        post_event(NoValidEvent()); 
}

AlignState::AlignState(my_context ctx) : my_base(ctx) 
{
    ROS_INFO("Into AlignState");

    PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloud();
    PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloud();
    PointCloudRGB::Ptr aligned_cloud(new PointCloudRGB);

    double radius_factor = 1.4;    
    ros::param::get("/radius_factor", radius_factor);

    int align_method = 1;
    ros::param::get("/align_method", align_method);

    // Perform initial alignment
    InitialAlignment initial_alignment(cad_cloud, scan_cloud);
    
    initial_alignment.setMethod((AlignmentMethod)align_method);
    initial_alignment.setRadiusFactor(radius_factor);

    initial_alignment.run(); 
    initial_alignment.getAlignedCloud(aligned_cloud);

    // store transform value on StateMachine
    context<StateMachine>().setCloudTransform(initial_alignment.getRigidTransform());
    ROS_INFO("Transform result from pre-alignment");
    Utils::printTransform(initial_alignment.getRigidTransform());
    
    if (Utils::isValidCloud(aligned_cloud))
        post_event(ValidEvent(aligned_cloud));
    else
        post_event(NoValidEvent());    
}

GICPState::GICPState(my_context ctx) : my_base(ctx) 
{
    ROS_INFO("Into GICPState");

    PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloud();
    PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloud();
    PointCloudRGB::Ptr aligned_cloud(new PointCloudRGB);

    bool compute_cov = false;    
    ros::param::get("/gicp_with_covariances", compute_cov);

    GICPAlignment gicp_alignment(cad_cloud, scan_cloud, compute_cov);
    gicp_alignment.run();
    gicp_alignment.getAlignedCloud(aligned_cloud);

    if (!Utils::isValidTransform(gicp_alignment.getFineTransform()))    
    {
        ROS_ERROR("Nan on transform");
        post_event(NoValidEvent());
    }
    else
    {
        // update and store transform value on StateMachine
        ROS_INFO("Updated transform:");
        Eigen::Matrix4f final_transform = gicp_alignment.getFineTransform() * context<StateMachine>().getCloudTransform();
        Utils::printTransform(final_transform);
        context<StateMachine>().setCloudTransform(final_transform);

        if (Utils::isValidCloud(aligned_cloud))
            post_event(ValidEvent(aligned_cloud));  
        else
            post_event(NoValidEvent());
    }
}

FODDetectionState::FODDetectionState(my_context ctx) : my_base(ctx)
{
    PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloudFiltered();
    PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloudFiltered();
    PointCloudRGB::Ptr substracted_cloud(new PointCloudRGB);

    // apply tf to source orig cloud
    Eigen::Matrix4f final_transform = context<StateMachine>().getCloudTransform();
    Utils::printTransform(final_transform);
    pcl::transformPointCloud(*scan_cloud, *scan_cloud, final_transform);

    double voxelize_factor = 3;
    ros::param::get("/voxelize_factor", voxelize_factor);
    ROS_INFO("Received from launch voxelize factor: %f", voxelize_factor);

    double th = 4e-3 * voxelize_factor;
    Filter::removeFromCloud(scan_cloud, cad_cloud, th, substracted_cloud);
    
    double min_fod_points = 3;
    ros::param::get("/min_fod_points", min_fod_points);

    int num_of_fods = 0;
    std::vector<PointCloudRGB::Ptr> fods_cloud_array;

    if (Utils::isValidCloud(substracted_cloud))
    {   
        // Viewer::visualizePointCloud<pcl::PointXYZRGB>(substracted_cloud);
        FODDetector fod_detector(substracted_cloud, th*100, min_fod_points);
        fod_detector.clusterPossibleFODs();

        // create a pointcloud for each cluster
        num_of_fods = fod_detector.fodIndicesToPointCloud(fods_cloud_array);
    }
    else
    {
        // an empty substracted cloud means no fods on piece
        ROS_INFO("No difference between scan and CAD cloud");
        substracted_cloud = nullptr;
        fods_cloud_array.push_back(substracted_cloud);
    }
    ROS_INFO("detected %d fods", num_of_fods);
    post_event(ValidFODEvent(num_of_fods, fods_cloud_array));  

    // post_event(NoValidEvent()); TODO
} 


PublishState::PublishState(my_context ctx) : my_base(ctx) 
{
    sensor_msgs::PointCloud2 cad_cloud_msg, scan_cloud_msg;
    
    // PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloud();
    // PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloud();
    PointCloudRGB::Ptr scan_cloud = context<StateMachine>().getSourceCloudFiltered();
    PointCloudRGB::Ptr cad_cloud = context<StateMachine>().getTargetCloudFiltered();

    // Convert cad cloud and aligned cloud to ROS data type
    Utils::cloudToROSMsg(scan_cloud, scan_cloud_msg);
    Utils::cloudToROSMsg(cad_cloud, cad_cloud_msg);

    // Publish clouds
    ROS_INFO("Publishing");
    context<StateMachine>().target_pub_.publish(cad_cloud_msg); 
    context<StateMachine>().source_pub_.publish(scan_cloud_msg);

    // If detected fods on scan cloud store them
    int n_fods = context<StateMachine>().getNFODs();
    if (n_fods>0)
    {
        std_msgs::Int16 n_fods_msg;
        PointCloudRGB::Ptr fods_cloud(new PointCloudRGB);
        
        std::vector<PointCloudRGB::Ptr> fods_cloud_array = context<StateMachine>().getFODCloudArray();
        for(int i=0; i<n_fods; i++)
        {
            /* sensor_msgs::PointCloud2 cloud_msg;
            Utils::cloudToROSMsg(fods_cloud_array[i], cloud_msg);
            context<StateMachine>().fods_pub_.publish(cloud_msg); */
            *fods_cloud  += *fods_cloud_array[i];
        }
        sensor_msgs::PointCloud2 cloud_msg;
        Utils::cloudToROSMsg(fods_cloud, cloud_msg);
        context<StateMachine>().fods_pub_.publish(cloud_msg);

        n_fods_msg.data = n_fods;
        context<StateMachine>().n_fods_pub_.publish(n_fods_msg);
    }    
}