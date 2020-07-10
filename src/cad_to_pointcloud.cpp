#include <cad_to_pointcloud.h>


CADToPointCloud::CADToPointCloud()
{}

CADToPointCloud::CADToPointCloud(std::string pointcloud_path, 
                                 std::string cad_file, 
                                 pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud)
{    
    setPCpath(pointcloud_path);

    if (CADToMesh(cad_file) == 0) // here we get _CAD_mesh
    {
        if(MeshToPointCloud(_CAD_mesh)==0) // here we get _CAD_cloud
        {
            pointcloud = _CAD_cloud;
        } 
    } 
}

CADToPointCloud::CADToPointCloud(std::string cad_file_path, 
                                 PointCloudRGB::Ptr cloud)
{    
    ROS_INFO("Converting file: %s.obj", cad_file_path.c_str());

    if (CADToMesh(cad_file_path) == 0) // here we get _CAD_mesh
    {
        if(MeshToPointCloud(_CAD_mesh)==0) // here we get _CAD_cloud
        {
            pcl::copyPointCloud(*_CAD_cloud, *cloud);
            ROS_INFO("File converted");
        } 
    } 
}

int CADToPointCloud::CADToMesh(std::string cad_file_path)
{
    pcl::PolygonMesh mesh;
    pcl::io::loadPolygonFile(cad_file_path,mesh);

    int len = mesh.cloud.row_step * mesh.cloud.height; 
    if (len == 0)
        return -1;

    _CAD_mesh = mesh;
    
    return 0;
    // visualizeMesh(_CAD_mesh);
}

int CADToPointCloud::MeshToPointCloud(pcl::PolygonMesh mesh)
{
    int SAMPLE_POINTS_ = 500000;
    float leaf_size = 0.01f;

    vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
    pcl::io::mesh2vtk(mesh, polydata1);


    // vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
    // readerQuery->SetFileName(inputfile.c_str());
    // readerQuery->Update();
    // polydata1 = readerQuery->GetOutput();

    // //make sure that the polygons are triangles!
    // vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    // triangleFilter->SetInputData(polydata1);
    // triangleFilter->Update();

    // vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    // triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    // triangleMapper->Update();
    // polydata1 = triangleMapper->GetInput();

    uniform_sampling(polydata1, SAMPLE_POINTS_, *_CAD_cloud);

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


/* Copyright of what's below: 

    pcl / tools / mesh_sampling.cpp

*/

void CADToPointCloud::uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples, pcl::PointCloud<pcl::PointXYZ> &cloud_out)
{
    polydata->BuildCells();
    vtkSmartPointer<vtkCellArray> cells = polydata->GetPolys();

    double p1[3], p2[3], p3[3], totalArea = 0;
    std::vector<double> cumulativeAreas(cells->GetNumberOfCells(), 0);
    size_t i = 0;
    vtkIdType npts = 0, *ptIds = NULL;
    for (cells->InitTraversal(); cells->GetNextCell(npts, ptIds); i++)
    {
        polydata->GetPoint(ptIds[0], p1);
        polydata->GetPoint(ptIds[1], p2);
        polydata->GetPoint(ptIds[2], p3);
        totalArea += vtkTriangle::TriangleArea(p1, p2, p3);
        cumulativeAreas[i] = totalArea;
    }

    cloud_out.points.resize(n_samples);
    cloud_out.width = static_cast<pcl::uint32_t>(n_samples);
    cloud_out.height = 1;

    for (i = 0; i < n_samples; i++)
    {
        Eigen::Vector4f p;
        randPSurface(polydata, &cumulativeAreas, totalArea, p);
        cloud_out.points[i].x = p[0];
        cloud_out.points[i].y = p[1];
        cloud_out.points[i].z = p[2];
    }
}


void CADToPointCloud::randPSurface(vtkPolyData *polydata, std::vector<double> *cumulativeAreas, double totalArea, Eigen::Vector4f &p)
{
    float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

    std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
    vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

    double A[3], B[3], C[3];
    vtkIdType npts = 0;
    vtkIdType *ptIds = NULL;
    polydata->GetCellPoints(el, npts, ptIds);
    polydata->GetPoint(ptIds[0], A);
    polydata->GetPoint(ptIds[1], B);
    polydata->GetPoint(ptIds[2], C);
    randomPointTriangle(float(A[0]), float(A[1]), float(A[2]),
                        float(B[0]), float(B[1]), float(B[2]),
                        float(C[0]), float(C[1]), float(C[2]), p);
}

double CADToPointCloud::uniform_deviate(int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}
 
void CADToPointCloud::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1, float c2, float c3, Eigen::Vector4f &p)
{
    float r1 = static_cast<float>(uniform_deviate(rand()));
    float r2 = static_cast<float>(uniform_deviate(rand()));
    float r1sqr = sqrtf(r1);
    float OneMinR1Sqr = (1 - r1sqr);
    float OneMinR2 = (1 - r2);
    a1 *= OneMinR1Sqr;
    a2 *= OneMinR1Sqr;
    a3 *= OneMinR1Sqr;
    b1 *= OneMinR2;
    b2 *= OneMinR2;
    b3 *= OneMinR2;
    c1 = r1sqr * (r2 * c1 + b1) + a1;
    c2 = r1sqr * (r2 * c2 + b2) + a2;
    c3 = r1sqr * (r2 * c3 + b3) + a3;
    p[0] = c1;
    p[1] = c2;
    p[2] = c3;
    p[3] = 0;
}