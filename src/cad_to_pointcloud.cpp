#include <cad_to_pointcloud.h>


CADToPointCloud::CADToPointCloud()
{}

CADToPointCloud::CADToPointCloud(std::string pointcloud_path, 
                                 std::string cad_file, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud, 
                                 bool big_file)
{    
    setPCpath(pointcloud_path);

    if (CADToMesh(cad_file) == 0) // here we get _CAD_mesh
    {
        if (big_file) MeshToPointCloud(_CAD_mesh); // here we get _CAD_cloud
        else 
        {
            if(MeshToPointCloud(cad_file)==0) // here we get _CAD_cloud
            {
                pointcloud = _CAD_cloud;
            } 
        }
    } 
}

int CADToPointCloud::CADToMesh(std::string filename)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile(_pc_path+filename,mesh);

    int len = mesh.cloud.row_step * mesh.cloud.height; 
    if (len == 0)
        return -1;

    _CAD_mesh = mesh;
    
    return 0;
    // visualizeMesh(_CAD_mesh);
}

int CADToPointCloud::MeshToPointCloud(std::string filename)
{
    // TODO
    // deshacer esta forma tan cutre
    std::string inputfile = _pc_path+filename;
    std::string outputfile = _pc_path+"tmp.pcd";

    std::string cmd = "pcl_mesh_sampling "+ inputfile + " " + outputfile + " -n_samples 500000";
    system(cmd.c_str());

    pcl::io::loadPCDFile<pcl::PointXYZ> (outputfile, *_CAD_cloud);
    if (_CAD_cloud->size()<=0)
        return -1;

    cmd = "rm " + outputfile;
    system(cmd.c_str());

    return 0;
}

int CADToPointCloud::MeshToPointCloud(pcl::PolygonMesh mesh)
{
    pcl::fromPCLPointCloud2(mesh.cloud, *_CAD_cloud);
    // TODO esta funcion esta muy limitada. 
    // Hay que intentar reproducir el comportamiento de este programa:
    // pcl_mesh_sampling input.obj output.pcd    

    // visualizePointCloud(_CAD_cloud);
    return 0;
}

int CADToPointCloud::MeshToROSPointCloud(pcl::PolygonMesh mesh)
{
    pcl_conversions::fromPCL( mesh.cloud, _CAD_cloud_msg);

    // Hay una función de pcl_ros muy util:
    // rosrun pcl_ros pcd_to_pointcloud input.pcd periodo _frame_id:=/world
    return 0;
}

void CADToPointCloud::setPCpath(std::string path)
{
    _pc_path = path;
}