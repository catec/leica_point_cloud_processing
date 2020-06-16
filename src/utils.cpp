#include <utils.h>


Utils::Utils()
{
    _pc_path = getPCpath();
}

std::string Utils::getPCpath()
{
    std::string pkg_path = ros::package::getPath("leica_scanstation");

	_pc_path = pkg_path + "/pointclouds/";
	return _pc_path;
}

bool Utils::getNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       double normal_radius,
                       pcl::PointCloud<pcl::Normal>::Ptr &normals)
{
    ROS_INFO("Computing normals with radius: %f",normal_radius);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    ne.setRadiusSearch(normal_radius);
    ne.compute(*normals);

    // As we compute normal for each point in cloud, must have same size
    bool success = normals->points.size()==cloud->points.size() ? true : false;

    return success;
}

void Utils::cloudToXYZRGB(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,
                          int R, int G, int B)
{
    pcl::copyPointCloud(*cloud,*cloud_rgb);
    for (size_t i = 0; i < cloud_rgb->points.size(); i++)
    {
        cloud_rgb->points[i].r = R;
        cloud_rgb->points[i].g = G;
        cloud_rgb->points[i].b = B;
    }
}


double Utils::computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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

double Utils::computeCloudResolution(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    double res = 0.0;
    int n_points = 0;
    int nres;
    std::vector<int> indices (2);
    std::vector<float> sqr_distances (2);
    pcl::search::KdTree<pcl::PointXYZRGB> tree;
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