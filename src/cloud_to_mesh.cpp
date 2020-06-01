#include <stdlib.h> 
// #include "pcl/common/angles.h"
// #include "pcl/sample_consensus/method_types.h"
// #include "pcl/sample_consensus/model_types.h"
// #include <pcl/filters/crop_box.h>
#include <pcl/surface/organized_fast_mesh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/gp3.h>
#include <pcl_ros/point_cloud.h> 
#include <cad_to_pointcloud.h>

#define NORMAL_RADIUS 0.1

int main(int argc, char** argv)
{
    ros::init(argc, argv, "CloudToMesh");

    std::string file_name = "";
    double radius_search;
    if (argc<2)
    {
        ROS_ERROR("Please specify file_name");
        return 0;
    }
    else
    {
        file_name = argv[1];
        if (file_name.find(".")!=std::string::npos)
        {
            ROS_ERROR("Please specify file_name without extension");
            return 0;
        }
        if (argc<3)
        {
            radius_search = NORMAL_RADIUS;
            ROS_WARN("Threshold not specified. Set to default: %f",radius_search);
        }
        else 
        {
            radius_search = atof(argv[2]);
            if (radius_search < 0)
            {
                ROS_ERROR("Please specify non-negative threshold");
                return 0;
            }
        }
    }

    ROS_INFO("open file");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    CADToPointCloud cad_to_pointcloud;
    std::string f = cad_to_pointcloud._pc_path + file_name + ".pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ> (f, *cloud);
    // cad_to_pointcloud.visualizePointCloud(cloud, cad_to_pointcloud.PINK);

    ROS_INFO("getting normals");
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud);
    ne.setRadiusSearch(NORMAL_RADIUS);
    ne.compute(*normals);

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);

    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud(cloud_with_normals);

    ROS_INFO("computing Greedy triangulation");
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh;
    gp3.setSearchRadius(NORMAL_RADIUS*1.5); // 20% higher than normal radius
    // Set typical values for the parameters
    gp3.setMu(3);
    gp3.setMaximumNearestNeighbors(100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod(tree2);
    gp3.reconstruct(mesh);

    cad_to_pointcloud.visualizeMesh(mesh);
    
    ROS_INFO("saving");
    f = cad_to_pointcloud._pc_path + file_name + "_reconstructed.stl";
    pcl::io::savePolygonFileSTL(f,mesh);


    return 0;
}