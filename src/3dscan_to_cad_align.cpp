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
* Large scans key issue:

* To find and appropiate descriptor but also an appropiate scale, for computing proccess

* One option is to look for descriptors that proves to be distintive at the scale 
* and persistent over multiple scales 
**/

const Eigen::Vector4f downsampling_leaf_size(0.05f, 0.05f, 0.05f, 0.0f);

class PointCloudAlignment 
{
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;
        Eigen::Matrix4f transform;
        Eigen::Matrix4d fine_transform;

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
                               pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud);
        
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

/*     pcl::visualization::CloudViewer viewer("Downsampled:");
    viewer.showCloud(cloud_downsampled, "Downsampled");
    ROS_INFO("Waiting for the user to quit the visualization window");
    while (!viewer.wasStopped(50)) {
    } */
}

void PointCloudAlignment::getKeypointsAndFeatures(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                                  pcl::PointCloud<pcl::Normal>::Ptr normals,
                                                  pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_cloud,
                                                  pcl::PointCloud<pcl::FPFHSignature33>::Ptr features)
{
    // SELECTED DESCRIPTOR: FPFH
    ROS_INFO("1. Set descriptor FPFH");
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>::Ptr  
    fest(new pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    fest->setInputCloud(cloud);
    fest->setInputNormals(normals);
    // Use all neighbors in a sphere of radius 5cm
    // IMPORTANT: the radius has to be larger than the radius used to estimate the surface normals!!!
    fest->setRadiusSearch (0.5);
    fest->setSearchMethod(tree);

    // ROS_INFO("2. Computing features");
    // pcl::PointCloud<pcl::FPFHSignature33>::Ptr features (new pcl::PointCloud<pcl::FPFHSignature33> ());
    // // fest->compute(*features);

    ROS_INFO("3. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>); // Interest points
    std::vector<float> scale_values = { 0.5f, 1.0f, 1.5f }; //pre-selected 
    fper.setScalesVector(scale_values);
    fper.setAlpha(1.3f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);

    ROS_INFO("4. Extracting keypoints");
    fper.determinePersistentFeatures(*features, keypoints);

    ROS_INFO("keypoints: %zu",keypoints->size());
    ROS_INFO("features: %zu",features->size());

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(keypoints);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr persistent_features_locations(new pcl::PointCloud<pcl::PointXYZ>());
    extract_indices_filter.filter(*keypoints_cloud);

/*     pcl::visualization::CloudViewer viewer("Viewer for keypoints");
    viewer.showCloud(keypoints_cloud, "persistent features");
    ROS_INFO("Persistent features have been computed. Waiting for the user to quit the visualization window");
    while (!viewer.wasStopped(50)) {
    } */
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
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch(0.03);
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

    pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(source_keypoints);
    rejector.setInputTarget(target_keypoints);
    rejector.setInlierThreshold(0.5);
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
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud)
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
    gicp.align(*source_cloud);
 
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

    ROS_INFO("Getting pointclouds to align");
    CADToPointCloud cad_to_pointcloud = CADToPointCloud("conjunto_estranio.obj", cad_pc, false);
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio_no_floor.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

/*     // Defining a rotation matrix and translation vector
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
    double theta = M_PI / 8;  // The angle of rotation in radians
    transformation_matrix (0, 0) = std::cos (theta);
    transformation_matrix (0, 1) = -sin (theta);
    transformation_matrix (1, 0) = sin (theta);
    transformation_matrix (1, 1) = std::cos (theta);
    // A translation on Z axis (0.4 meters)
    transformation_matrix (2, 3) = 0.4;
    pcl::transformPointCloud(*cad_pc,*scan_pc,transformation_matrix);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]"); // just for print
    std::cout << transformation_matrix.format(CleanFmt) << std::endl; */

    ROS_INFO("Downsampling...");
    const Eigen::Vector4f small_leaf_size(0.03f, 0.03f, 0.03f, 0.0f);
    const Eigen::Vector4f big_leaf_size(0.1f, 0.1f, 0.1f, 0.0f);
    point_cloud_alignment.downsampleCloud(cad_pc,cad_pc_downsampled,small_leaf_size);
    point_cloud_alignment.downsampleCloud(scan_pc,scan_pc_downsampled,small_leaf_size); 
    ROS_INFO("Computing normals...");
    point_cloud_alignment.getNormals(cad_pc_downsampled,cad_normals);
    point_cloud_alignment.getNormals(scan_pc_downsampled,scan_normals);
    cad_to_pointcloud.visualizePointCloud(cad_pc_downsampled,cad_to_pointcloud.RED);
    cad_to_pointcloud.addNormalsToVisualizer(cad_pc_downsampled,cad_normals,"cad_normals");
    cad_to_pointcloud.addPCToVisualizer(scan_pc_downsampled,cad_to_pointcloud.PINK,"scan");
    cad_to_pointcloud.addNormalsToVisualizer(scan_pc_downsampled,scan_normals,"scan_normals");
    ROS_INFO("Computing keypoints...");
    point_cloud_alignment.getKeypointsAndFeatures(cad_pc_downsampled,cad_normals,cad_keypoints,cad_features);
    cad_to_pointcloud.addPCToVisualizer(cad_keypoints,cad_to_pointcloud.GREEN,"cad_key");
    point_cloud_alignment.getKeypointsAndFeatures(scan_pc_downsampled,scan_normals,scan_keypoints,scan_features);
    cad_to_pointcloud.addPCToVisualizer(scan_keypoints,cad_to_pointcloud.GREEN,"scan_key");
    ROS_INFO("Getting transform...");
    point_cloud_alignment.initialAlingment(scan_features,cad_features,scan_keypoints,cad_keypoints);
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]"); // just for print
    std::cout << point_cloud_alignment.transform.format(CleanFmt) << std::endl;
    ROS_INFO("Applying transform...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*scan_pc_downsampled,*scan_aligned,point_cloud_alignment.transform);
    cad_to_pointcloud.addPCToVisualizer(scan_aligned,cad_to_pointcloud.BLUE,"scan_tf");
    ROS_INFO("Computing iterative Algorithm to get fine registration...");
    point_cloud_alignment.fine_registration(scan_aligned,cad_pc_downsampled);
    cad_to_pointcloud.addPCToVisualizer(scan_aligned,cad_to_pointcloud.ORANGE,"scan_tf_fine");
    ROS_INFO("Applying fine transform...");
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_fine_aligned(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*scan_pc_downsampled,*scan_fine_aligned,point_cloud_alignment.fine_transform);
 /*  
    // visualization
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cad_cloud_rgb(cad_pc_downsampled, 0, 0, 255); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scancad_pc_downsampled(scan_pc_downsampled, 255, 255, 255); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scan_fine_cloud_rgb(scan_aligned, 0, 255, 0);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> scan_fine_cloud_rgb(scan_fine_aligned, 0, 255, 0);

    pcl::visualization::PCLVisualizer viewer;
    viewer.setBackgroundColor(0,0,0);
    viewer.addCoordinateSystem(1.0);
    viewer.addPointCloud<pcl::PointXYZ>(cad_pc_downsampled, cad_cloud_rgb, "cad");
    viewer.addPointCloud<pcl::PointXYZ>(scan_pc_downsampled, scancad_pc_downsampled, "cad2");
    viewer.addPointCloud<pcl::PointXYZ>(scan_aligned, scan_cloud_rgb, "scan");
    viewer.addPointCloud<pcl::PointXYZ>(scan_fine_aligned, scan_fine_cloud_rgb, "scan fine");
    viewer.initCameraParameters();
    ROS_INFO("Waiting for the user to quit the visualization window");
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    } 
*/

    ROS_INFO("end");

    return 0;
}