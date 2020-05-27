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

bool next_iteration=false;

class PointCloudAlignment 
{
    public:
        PointCloudAlignment() {};
        ~PointCloudAlignment() {};

        pcl::PointCloud<pcl::PointXYZ> pc;
        Eigen::Matrix4f transform, fine_transform;
        double _normal_radius, _feature_radius,_inlier_threshold;
        bool _next_iteration;
        pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;

        void printTransform(Eigen::Matrix4f transform);
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
                               pcl::PointCloud<pcl::PointXYZ>::Ptr transf_cloud,
                               int iterations);
        
        void getCovariances(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                            boost::shared_ptr< std::vector<Eigen::Matrix3d, 
                            Eigen::aligned_allocator<Eigen::Matrix3d>> > covs);

};


void PointCloudAlignment::downsampleCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled,
                                          const Eigen::Vector4f downsampling_leaf_size)
{
    ROS_INFO("Pointcloud size before downsampling: %zu",cloud->points.size());
    pcl::VoxelGrid<pcl::PointXYZ> downsampling_filter;
    downsampling_filter.setInputCloud(cloud);
    downsampling_filter.setLeafSize(downsampling_leaf_size);
    downsampling_filter.filter(*cloud_downsampled);
    ROS_INFO("Pointcloud size after downsampling: %zu",cloud_downsampled->points.size());
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

    return success;
}

Eigen::Vector3f point2vector(pcl::PointXYZ p)
{
    return Eigen::Vector3f(p.data);
}

double computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  double res = 0.0;
  int n_points = 0;
  int nres;
  std::vector<int> indices (2);
  std::vector<float> sqr_distances (2);
  pcl::search::KdTree<pcl::PointXYZ> tree;
  tree.setInputCloud(cloud);

  for (std::size_t i = 0; i < cloud->size (); ++i)
  {
    if (! std::isfinite ((*cloud)[i].x))
    {
      continue;
    }
    //Considering the second neighbor since the first is the point itself.
    nres = tree.nearestKSearch (i, 2, indices, sqr_distances);
    if (nres == 2)
    {
      res += sqrt (sqr_distances[1]);
      ++n_points;
    }
  }
  if (n_points != 0)
  {
    res /= n_points;
  }
  return res;
}

std::vector<float> getScaleValues(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    std::vector<float> scale_values;
    // pcl::PointXYZ minPt, maxPt;
    // pcl::getMinMax3D(*cloud, minPt, maxPt);
    // Eigen::Vector3f diff = point2vector(maxPt) - point2vector(minPt);
    // double x_resolution = diff[0] / sqrt(cloud->size());
    // double y_resolution = diff[1] / sqrt(cloud->size());
    // double z_resolution = diff[2] / sqrt(cloud->size());
    // scale_values.push_back((float)x_resolution);
    // scale_values.push_back((float)y_resolution);
    // scale_values.push_back((float)z_resolution);

    double cloud_resolution = computeCloudResolution(cloud);
    for (int i=1; i<=3; i++)
    {
        // scale_values.push_back((float)_inlier_threshold*i);
        scale_values.push_back((float)cloud_resolution*i);
    }

    return scale_values;
}

void printScaleValues(std::vector<float> scale_values)
{
    ROS_INFO("scale:");
    for (int i=0; i<scale_values.size(); i++)
    {
        ROS_INFO("%f",scale_values[i]);
    }
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

    // -- 1
    ROS_INFO("2. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper;
    boost::shared_ptr<std::vector<int> > keypoints1(new std::vector<int>); // Interest points
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features1 (new pcl::PointCloud<pcl::FPFHSignature33>);
    std::vector<float> scale_values1;
    scale_values1 = getScaleValues(cloud);
    printScaleValues(scale_values1);

    fper.setScalesVector(scale_values1);
    fper.setAlpha(1.3f);
    fper.setFeatureEstimator(fest);
    fper.setDistanceMetric(pcl::CS);

    ROS_INFO("3. Extracting keypoints");
    fper.determinePersistentFeatures(*features1, keypoints1);
    ROS_INFO("keypoints1: %zu", keypoints1->size());   

    // -- 2
    ROS_INFO("2. Define feature persistence");
    pcl::MultiscaleFeaturePersistence<pcl::PointXYZ, pcl::FPFHSignature33> fper2;
    boost::shared_ptr<std::vector<int> > keypoints2(new std::vector<int>); // Interest points
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr features2 (new pcl::PointCloud<pcl::FPFHSignature33>);
    std::vector<float> scale_values2;
    scale_values2.push_back(0.5f);
    scale_values2.push_back(1.0f);   
    scale_values2.push_back(1.5f);
    printScaleValues(scale_values2);

    fper2.setScalesVector(scale_values2);
    fper2.setAlpha(1.3f);
    fper2.setFeatureEstimator(fest);
    fper2.setDistanceMetric(pcl::CS);

    ROS_INFO("3. Extracting keypoints");
    fper2.determinePersistentFeatures(*features2, keypoints2);
    ROS_INFO("keypoints2: %zu", keypoints2->size());   

    boost::shared_ptr<std::vector<int> > keypoints(new std::vector<int>);
    // keypoints = keypoints1;
    keypoints->insert(keypoints->end(),keypoints1->begin(),keypoints1->end());
    keypoints->insert(keypoints->end(),keypoints2->begin(),keypoints2->end());
    features->insert(features->end(),features1->begin(),features1->end());
    features->insert(features->end(),features2->begin(),features2->end());


    ROS_INFO("keypoints: %zu", keypoints->size());   
    bool success = keypoints->size()==features->size() ? true : false;

    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(keypoints);
    extract_indices_filter.filter(*keypoints_cloud);
}


void PointCloudAlignment::initialAlingment(pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features,
                                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr source_keypoints,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr target_keypoints)
{
    ROS_INFO("4. Use descriptor FPFH to compute correspondences");
    pcl::CorrespondencesPtr correspondences(new pcl::Correspondences);
    pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> cest;
    cest.setInputSource(source_features);
    cest.setInputTarget(target_features);
    cest.determineCorrespondences(*correspondences);

    ROS_INFO("5. Get correspondences with inlier threshold: %f",_inlier_threshold);
    pcl::CorrespondencesPtr corr_filtered(new pcl::Correspondences);
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZ> rejector;
    rejector.setInputSource(source_keypoints);
    rejector.setInputTarget(target_keypoints);
    rejector.setInlierThreshold(_inlier_threshold);
    rejector.setMaximumIterations(100000);
    rejector.setRefineModel(false);
    rejector.setInputCorrespondences(correspondences);;
    rejector.getCorrespondences(*corr_filtered);
    
    ROS_INFO("6. Get rigid transformation: ");
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> trans_est;
    trans_est.estimateRigidTransformation(*source_keypoints,*target_keypoints, 
                                          *corr_filtered, transform);
    printTransform(transform);
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
    // pcl::features::computeApproximateNormals(*cloud, mesh->polygons, *normals); // NOT WORKING
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    // alternative way of getting normals
    getNormals(cloud,normals);
    // filter NaN values
    boost::shared_ptr<std::vector<int> > indices(new std::vector<int>); // Interest points
    pcl::removeNaNFromPointCloud(*cloud,*cloud,*indices);
    pcl::removeNaNNormalsFromPointCloud(*normals,*normals,*indices);
    pcl::ExtractIndices<pcl::PointXYZ> extract_indices_filter;
    extract_indices_filter.setInputCloud(cloud);
    extract_indices_filter.setIndices(indices);
    extract_indices_filter.filter(*cloud);

    // get covariances
    pcl::features::computeApproximateCovariances(*cloud, *normals, *covs);
    bool success = normals->points.size()==covs->size() ? true : false;
}

void PointCloudAlignment::fine_registration(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr transf_cloud,
                                            int iterations)
{
    ROS_INFO("7. Extract covariances from clouds");
    boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > 
            source_covariances(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>);
    boost::shared_ptr< std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>> > 
            target_covariances(new std::vector<Eigen::Matrix3d, Eigen::aligned_allocator<Eigen::Matrix3d>>);

    getCovariances(source_cloud,source_covariances);
    getCovariances(target_cloud,target_covariances);
    
    ROS_INFO("8. Perform GICP with %d iterations", iterations);
    ros::Time begin = ros::Time::now();
    // setup Generalized-ICP
    // pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
    gicp.setMaxCorrespondenceDistance(0.5);
    gicp.setMaximumIterations(iterations); // no encuentro el numero maximo
    gicp.setEuclideanFitnessEpsilon(1e-5);
    gicp.setTransformationEpsilon(1e-5);
    gicp.setRANSACOutlierRejectionThreshold(0.5);
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);
    gicp.setSourceCovariances(source_covariances);
    gicp.setTargetCovariances(target_covariances);
    // run registration and get transformation
    gicp.align(*source_cloud);

    ros::Duration exec_time = ros::Time::now()-begin;
    ROS_INFO("GICP time: %lf s",exec_time.toSec());

    if (gicp.hasConverged())
    {
        ROS_INFO("Converged in %f",gicp.getFitnessScore());
        fine_transform = gicp.getFinalTransformation();
        ROS_INFO("9. Transform result of GICP: ");
        printTransform(fine_transform);
        gicp.setMaximumIterations(10); // for future iterations
    }
    else 
    {
        ROS_ERROR("NO CONVERGE");
    }
}


void PointCloudAlignment::printTransform(Eigen::Matrix4f transform)
{
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << transform.format(CleanFmt) << std::endl;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
  if (event.getKeySym () == "space" && event.keyDown ())
    next_iteration = true;
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
    double cnst,n_r,f_r,th;
    int iter;
    bool NORMALS=true,KEYPOINTS=true,TRANSFORM=true,ICP=true,VISUALIZE=true;
    if (!nh.getParam("/PointCloudAlignment/small_leaf_size", small_leaf))   return 0;
    if (!nh.getParam("/PointCloudAlignment/big_leaf_size", big_leaf))       return 0;
    if (!nh.getParam("/PointCloudAlignment/normal_r", n_r))       return 0;
    if (!nh.getParam("/PointCloudAlignment/feature_r", f_r))       return 0;
    if (!nh.getParam("/PointCloudAlignment/threshold", th))       return 0;
    if (!nh.getParam("/PointCloudAlignment/run_normals", NORMALS))          return 0;
    if (!nh.getParam("/PointCloudAlignment/run_keypoints", KEYPOINTS))      return 0;
    if (!nh.getParam("/PointCloudAlignment/run_transform", TRANSFORM))      return 0;
    if (!nh.getParam("/PointCloudAlignment/run_icp", ICP))                  return 0;
    if (!nh.getParam("/PointCloudAlignment/visualize", VISUALIZE))          return 0;
    if (!nh.getParam("/PointCloudAlignment/const", cnst))                   return 0;
    if (!nh.getParam("/PointCloudAlignment/iter", iter))                    return 0;
    // point_cloud_alignment._normal_radius = (big_leaf+small_leaf)*1.25  ; // 25% higher
    // point_cloud_alignment._feature_radius = point_cloud_alignment._normal_radius*1.20; // 20% higher
    // point_cloud_alignment._inlier_threshold = point_cloud_alignment._normal_radius*cnst;
    point_cloud_alignment._normal_radius = n_r;
    point_cloud_alignment._feature_radius = f_r;
    point_cloud_alignment._inlier_threshold = th;
    ROS_INFO("Iterations: %d",iter);
    
    // Pointclouds to be aligned:
    // cad_pc is the target pointcloud directly obtain from a part's cad
    // scan_pc is the source pointcloud created on gazebo with leica c5 simulator
    ROS_INFO("Getting pointclouds to align");
    // CADToPointCloud cad_to_pointcloud = CADToPointCloud("conjunto_estranio.obj", cad_pc, false);
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + "conjunto_estranio.pcd";
    std::string f2 = cad_to_pointcloud._pc_path + "conjunto_estranio_cad.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f2, *cad_pc);
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *scan_pc);

    ros::Time begin = ros::Time::now();

    ROS_INFO("Downsampling...");
    const Eigen::Vector4f small_leaf_size(small_leaf, small_leaf, small_leaf, 0.0f);
    const Eigen::Vector4f big_leaf_size(big_leaf, big_leaf, big_leaf, 0.0f);
    point_cloud_alignment.downsampleCloud(cad_pc,cad_pc_downsampled,small_leaf_size);
    point_cloud_alignment.downsampleCloud(scan_pc,scan_pc_downsampled,big_leaf_size); 

/*     // get inlier threshold with resolution
    double cad_res = computeCloudResolution(cad_pc_downsampled);
    double scan_res = computeCloudResolution(scan_pc_downsampled);
    ROS_INFO("CAD Resolution: %f",cad_res);
    ROS_INFO("SCAN Resolution: %f",scan_res);
    point_cloud_alignment._inlier_threshold = std::min(cad_res,scan_res); */

    if (!NORMALS) return 0;

    ROS_INFO("Computing normals...");
    point_cloud_alignment.getNormals(cad_pc_downsampled,cad_normals);
    point_cloud_alignment.getNormals(scan_pc_downsampled,scan_normals);
    if (VISUALIZE)
    {
        cad_to_pointcloud.visualizePointCloud(cad_pc_downsampled,cad_to_pointcloud.RED);
        // cad_to_pointcloud.addNormalsToVisualizer(cad_pc_downsampled,cad_normals,"cad_normals");
        cad_to_pointcloud.addPCToVisualizer(scan_pc_downsampled,cad_to_pointcloud.PINK,"scan");
        // cad_to_pointcloud.addNormalsToVisualizer(scan_pc_downsampled,scan_normals,"scan_normals");
    }


    if (!KEYPOINTS) return 0;

    ROS_INFO("Computing keypoints...");
    point_cloud_alignment.getKeypointsAndFeatures(cad_pc_downsampled,cad_normals,cad_keypoints,cad_features);
    point_cloud_alignment.getKeypointsAndFeatures(scan_pc_downsampled,scan_normals,scan_keypoints,scan_features);
    if (VISUALIZE)
    {
        cad_to_pointcloud.addPCToVisualizer(cad_keypoints,cad_to_pointcloud.GREEN,"cad_key");
        cad_to_pointcloud.addPCToVisualizer(scan_keypoints,cad_to_pointcloud.GREEN,"scan_key");
    }
    
    if (!TRANSFORM) return 0;

    ROS_INFO("Getting transform...");
    point_cloud_alignment.initialAlingment(scan_features,cad_features,scan_keypoints,cad_keypoints);

    ROS_INFO("Applying transform...");
    pcl::transformPointCloud(*scan_pc_downsampled,*scan_aligned,point_cloud_alignment.transform);
    if (VISUALIZE)
    {
        cad_to_pointcloud.addPCToVisualizer(scan_aligned,cad_to_pointcloud.BLUE,"scan_tf");         
    }

    if (!ICP) return 0;

    ROS_INFO("Computing iterative Algorithm to get fine registration...");
    point_cloud_alignment.fine_registration(scan_aligned,cad_pc_downsampled,scan_fine_aligned,iter);
    if (VISUALIZE)
    {
        cad_to_pointcloud.addPCToVisualizer(scan_aligned,cad_to_pointcloud.ORANGE,"scan_tf_fine");
        ROS_INFO("Check viewer");
    }

    ros::Duration exec_time = ros::Time::now() - begin;
    ROS_INFO("Total process time: %lf s",exec_time.toSec());

    // more iterations    
    pcl::visualization::PCLVisualizer viewer("GICP");
    viewer.setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb(cad_pc_downsampled, 255,255,255); 
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_rgb2(scan_aligned, 255,0,255); 
    viewer.addPointCloud<pcl::PointXYZ>(cad_pc_downsampled,cloud_rgb,"cloud");
    viewer.addPointCloud<pcl::PointXYZ>(scan_aligned,cloud_rgb2,"cloud2");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,2);
    viewer.addCoordinateSystem(1.0);
    viewer.initCameraParameters();
    ROS_INFO("On viewer press space to perform iteration");

    viewer.registerKeyboardCallback(&keyboardEventOccurred,(void*)NULL);

    ROS_INFO("Final transformation:");
    Eigen::Matrix4f final_transform = point_cloud_alignment.fine_transform * point_cloud_alignment.transform;
    point_cloud_alignment.printTransform(final_transform);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
        // The user pressed "space" :
        if (next_iteration)
        {
            ROS_INFO("Computing iteration...");
            // The Iterative Closest Point algorithm
            point_cloud_alignment.gicp.align(*scan_aligned);

            if (point_cloud_alignment.gicp.hasConverged()) 
            {
                ROS_INFO("Final transformation:");
                point_cloud_alignment.fine_transform = point_cloud_alignment.gicp.getFinalTransformation();
                final_transform = point_cloud_alignment.fine_transform * final_transform;
                point_cloud_alignment.printTransform(final_transform);
                // point_cloud_alignment.printTransform(point_cloud_alignment.fine_transform);
                ROS_INFO("Converged in %f",point_cloud_alignment.gicp.getFitnessScore());
                ROS_INFO("Update cloud");
                viewer.updatePointCloud(scan_aligned,cloud_rgb2,"cloud2");
            }
            else
            {
                ROS_ERROR("no converge");
            }
        }
        next_iteration = false;
    }

    ROS_INFO("the end");

    return 0;
}