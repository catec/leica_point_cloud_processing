#include "ros/ros.h"
#include "ros/package.h"
// #include "pcl_conversions/pcl_conversions.h"
// #include <pcl_ros/point_cloud.h> 
#include <pcl/io/vtk_lib_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/multiscale_feature_persistence.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/registration/gicp.h>
#include <pcl/features/from_meshes.h>
#include <cad_to_pointcloud.h>

/**
 * POINTCLOUDS:
    - target pointcloud: directly obtain from downsampling a part's cad
    - source pointcloud: result from scanning the same cad part in Gazebo with leica c5 simulator
 * WORKFLOW:
    1. Filter pointclouds to reduce number of points (VoxelGrid)
    2. Get normal for every point in both clouds (pcl::NormalEstimation)
    3. Extract features and keypoints from pointcloud and it's normals (FPFH descriptor and pcl::MultiscaleFeaturePersistence)
    4. Perform initial aligment as rigid transformation (pcl::CorrespondenceEstimation)
    5. Applied GICP to refine transformation (pcl::GeneralizedIterativeClosestPoint)
**/

class PointCloudAlignment 
{
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;
        Eigen::Matrix4f transform;
        Eigen::Matrix4d fine_transform;
        double _normal_radius, _feature_radius,_inlier_threshold;

        void printKeypoints(std::vector<int> const &input);
        bool getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                        pcl::PointCloud<pcl::Normal>::Ptr &normals);
        void getKeypointsAndFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                     pcl::PointCloud<pcl::Normal>::Ptr normals,
                                     pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud,
                                     pcl::PointCloud<pcl::FPFHSignature33>::Ptr features);

        void downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled,
                             const Eigen::Vector4f downsampling_leaf_size);

        void initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                              pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints);
        
        void fine_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr transf_cloud);
        
        void getCovariances(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > covs);

};

void PointCloudAlignment::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled,
                                          const Eigen::Vector4f downsampling_leaf_size)
{
    ROS_INFO("pc size before: %zu",cloud->points.size());
    pcl::VoxelGrid<pcl::PointXYZ> downsampling_filter;
    downsampling_filter.setInputCloud(cloud);
    downsampling_filter.setLeafSize(downsampling_leaf_size);
    downsampling_filter.filter(*cloud_downsampled);
    ROS_INFO("pc size after: %zu",cloud_downsampled->points.size());
}

void PointCloudAlignment::getKeypointsAndFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud,
                                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    // SELECTED DESCRIPTOR: FPFH
    ROS_INFO("1. Set descriptor FPFH with radius: %f",_feature_radius);
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr  
        fest(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    fest->setRadiusSearch(_feature_radius);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fest->setSearchMethod(tree);

    ROS_INFO("2. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>); // Interest points
    std::vector<float> scale_values = {0.5f,1.0f,1.5f}; //pre-selected 
    fper.setScalesVector(scale_values);
    fper.setAlpha(1.3f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);

    ROS_INFO("3. Extracting keypoints");
    fper.determinePersistentFeatures(*features, keypoints);

    ROS_INFO("keypoints: %zu",keypoints->size());
    ROS_INFO("features: %zu",features->size());

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(keypoints);
    extract_indices_filter.filter(*keypoints_cloud);
}

void PointCloudAlignment::printKeypoints(std::vector<int> const &input)
{
	for (int i = 0; i < input.size(); i++) {
		std::cout << input.at(i) << ' ';
	}
}

bool PointCloudAlignment::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                                     pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    ROS_INFO("Normal with radius: %f",_normal_radius);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius _normal_radius cm
    ne.setRadiusSearch(_normal_radius);
    ne.compute(*normals);

    // As we compute normal for each pointcloud, both should have same number of points
    bool success = normals->points.size()==cloud->points.size() ? true : false;
    ROS_INFO("normal size: %zu",normals->points.size());
    ROS_INFO("pc size: %zu",cloud->points.size());

    return success;
}

void PointCloudAlignment::initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints)
{
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
    cest.setInputSource(source_features);
    cest.setInputTarget(target_features);
    cest.determineCorrespondences(*correspondences);
    ROS_INFO("inlier threshold: %f",_inlier_threshold);
    pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(source_keypoints);
    rejector.setInputTarget(target_keypoints);
    rejector.setInlierThreshold(_inlier_threshold);
    rejector.setMaximumIterations(1000000);
    rejector.setRefineModel(false);
    rejector.setInputCorrespondences(correspondences);;
    rejector.getCorrespondences(*corr_filtered);
    
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    trans_est.estimateRigidTransformation(*source_keypoints,*target_keypoints, 
                                          *corr_filtered, transform);
}

void PointCloudAlignment::getCovariances(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                         boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > covs)
{
    // reconstruct meshes NOT WORKING
/*     pcl::PolygonMesh::Ptr mesh (new pcl::PolygonMesh);
    pcl::OrganizedFastMesh<pcl::PointXYZ> fast_mesh;
    fast_mesh.setInputCloud(cloud);
    fast_mesh.setMaxEdgeLength (15);
    fast_mesh.setTrianglePixelSize (20);
    fast_mesh.setTriangulationType(pcl::OrganizedFastMesh<pcl::PointXYZ>::TRIANGLE_ADAPTIVE_CUT);
    fast_mesh.reconstruct(*mesh);
    cad_to_pointcloud.visualizeMesh(*mesh); */    

    // compute normals and covariances for source and target
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    // pcl::features::computeApproximateNormals(*cloud, mesh->polygons, *normals); // NOT WORKING
    
    // alternative way of getting normals
    getNormals(cloud,normals);
    pcl::features::computeApproximateCovariances(*cloud, *normals, *covs);
    ROS_INFO("covs size: %zu",covs->size());
}

void PointCloudAlignment::fine_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr transf_cloud)
{
    boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > 
            source_covariances(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>);
    boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > 
            target_covariances(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>);

    getCovariances(source_cloud,source_covariances);
    getCovariances(target_cloud,target_covariances);

    // setup Generalized-ICP
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(1);
    gicp.setMaximumIterations(500); // no encuentro el numero maximo
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    // gicp.setSourceCovariances(source_covariances);
    // gicp.setTargetCovariances(target_covariances);
    // run registration and get transformation
    gicp.align(*transf_cloud);
 
    if (gicp.hasConverged())
    {
        ROS_INFO("Converged in %f",gicp.getFitnessScore());
        fine_transform = gicp.getFinalTransformation().cast<double>();
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]"); // just for print
        std::cout << fine_transform.format(CleanFmt) << std::endl;
    }
    else 
    {
        ROS_ERROR("NO CONVERGE");
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "PointCloudAlignment");
    ros::NodeHandle nh;
    ROS_INFO("PointCloudAlignment");

    PointCloudAlignment point_cloud_alignment;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_pc_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_pc_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr cad_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Normal>::Ptr scan_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cad_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr cad_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scan_features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_fine_aligned(new pcl::PointCloud<pcl::PointXYZ>);

    // PARAMETERS
    double small_leaf = 0.03f;
    double big_leaf = 0.05f;
    double cnst;
    bool NORMALS=true,KEYPOINTS=true,TRANSFORM=true,ICP=true;
    if (!nh.getParam("/PointCloudAlignment/small_leaf_size", small_leaf))   return 0;
    if (!nh.getParam("/PointCloudAlignment/big_leaf_size", big_leaf))       return 0;
    if (!nh.getParam("/PointCloudAlignment/run_normals", NORMALS))          return 0;
    if (!nh.getParam("/PointCloudAlignment/run_keypoints", KEYPOINTS))      return 0;
    if (!nh.getParam("/PointCloudAlignment/run_transform", TRANSFORM))      return 0;
    if (!nh.getParam("/PointCloudAlignment/run_icp", ICP))                  return 0;
    if (!nh.getParam("/PointCloudAlignment/const", cnst))                   return 0;
    point_cloud_alignment._normal_radius = (big_leaf+small_leaf)*1.25; // 25% higher
    point_cloud_alignment._feature_radius = point_cloud_alignment._normal_radius*1.20; // 20% higher
    point_cloud_alignment._inlier_threshold = point_cloud_alignment._normal_radius*cnst;

    // Pointclouds to be aligned:
    // cad_pc is the target pointcloud directly obtain from a part's cad
    // scan_pc is the source pointcloud created on gazebo with leica c5 simulator
    ROS_INFO("Getting pointclouds to align");
    CADToPointCloud cad_to_pointcloud = CADToPointCloud("conjunto_estranio.obj", cad_pc, false);
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

    ROS_INFO("Downsampling...");
    const Eigen::Vector4f small_leaf_size(small_leaf, small_leaf, small_leaf, 0.0f);
    const Eigen::Vector4f big_leaf_size(big_leaf, big_leaf, big_leaf, 0.0f);
    point_cloud_alignment.downsampleCloud(cad_pc,cad_pc_downsampled,small_leaf_size);
    point_cloud_alignment.downsampleCloud(scan_pc,scan_pc_downsampled,big_leaf_size); 

    if (!NORMALS) return 0;

    ROS_INFO("Computing normals...");
    point_cloud_alignment.getNormals(cad_pc_downsampled,cad_normals);
    point_cloud_alignment.getNormals(scan_pc_downsampled,scan_normals);
    cad_to_pointcloud.visualizePointCloud(cad_pc_downsampled,cad_to_pointcloud.RED);
    cad_to_pointcloud.addNormalsToVisualizer(cad_pc_downsampled,cad_normals,"cad_normals");
    cad_to_pointcloud.addPCToVisualizer(scan_pc_downsampled,cad_to_pointcloud.PINK,"scan");
    cad_to_pointcloud.addNormalsToVisualizer(scan_pc_downsampled,scan_normals,"scan_normals");

    if (!KEYPOINTS) return 0;

    ROS_INFO("Computing keypoints...");
    point_cloud_alignment.getKeypointsAndFeatures(cad_pc_downsampled,cad_normals,cad_keypoints,cad_features);
    point_cloud_alignment.getKeypointsAndFeatures(scan_pc_downsampled,scan_normals,scan_keypoints,scan_features);
    cad_to_pointcloud.addPCToVisualizer(cad_keypoints,cad_to_pointcloud.GREEN,"cad_key");
    cad_to_pointcloud.addPCToVisualizer(scan_keypoints,cad_to_pointcloud.GREEN,"scan_key");

    if (!TRANSFORM) return 0;

    ROS_INFO("Getting transform...");
    point_cloud_alignment.initialAlingment(scan_features,cad_features,scan_keypoints,cad_keypoints);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]"); // just for print
    std::cout << point_cloud_alignment.transform.format(CleanFmt) << std::endl;
    ROS_INFO("Applying transform...");
    pcl::transformPointCloud(*scan_pc_downsampled,*scan_aligned,point_cloud_alignment.transform);
    cad_to_pointcloud.addPCToVisualizer(scan_aligned,cad_to_pointcloud.BLUE,"scan_tf");

    if (!ICP) return 0;

    ROS_INFO("Computing iterative Algorithm to get fine registration...");
    point_cloud_alignment.fine_registration(scan_aligned,cad_pc_downsampled,scan_fine_aligned);
    cad_to_pointcloud.addPCToVisualizer(scan_fine_aligned,cad_to_pointcloud.ORANGE,"scan_tf_fine");
    ROS_INFO("Check viewer");
    
    ROS_INFO("the end");

    return 0;
}