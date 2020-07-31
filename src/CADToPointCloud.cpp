#include <CADToPointCloud.h>

CADToPointCloud::CADToPointCloud()
{
}

CADToPointCloud::CADToPointCloud(const std::string& cad_path, const std::string& cad_file, PointCloudXYZ::Ptr cloud)
{
  setPCpath(cad_path);

  if (CADToMesh(cad_path + cad_file) == 0)  // here we get _CAD_mesh
  {
    if (MeshToPointCloud(_CAD_mesh) == 0)  // here we get _CAD_cloud
    {
      pcl::copyPointCloud(*_CAD_cloud, *cloud);
      ROS_INFO("File converted");
    }
  }
}

CADToPointCloud::CADToPointCloud(const std::string& cad_file_path, PointCloudRGB::Ptr cloud)
{
  ROS_INFO("Converting file: %s", cad_file_path.c_str());

  if (CADToMesh(cad_file_path) == 0)  // here we get _CAD_mesh
  {
    if (MeshToPointCloud(_CAD_mesh) == 0)  // here we get _CAD_cloud
    {
      pcl::copyPointCloud(*_CAD_cloud, *cloud);
      ROS_INFO("File converted");
    }
  }
}

int CADToPointCloud::CADToMesh(const std::string& cad_file_path)
{
  pcl::PolygonMesh mesh;
  pcl::io::loadPolygonFile(cad_file_path, mesh);

  int len = mesh.cloud.row_step * mesh.cloud.height;
  if (len == 0)
    return -1;

  *_CAD_mesh = mesh;

  return 0;
}

int CADToPointCloud::MeshToPointCloud(pcl::PolygonMesh::Ptr mesh)
{
  int SAMPLE_POINTS_ = 500000;
  float leaf_size = 0.01f;

  vtkSmartPointer<vtkPolyData> polydata1 = vtkSmartPointer<vtkPolyData>::New();
  pcl::io::mesh2vtk(*mesh, polydata1);

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

int CADToPointCloud::MeshToROSPointCloud(pcl::PolygonMesh::Ptr mesh)
{
  pcl_conversions::fromPCL(mesh->cloud, _CAD_cloud_msg);
  _CAD_cloud_msg.header.frame_id = Utils::_frame_id;
  _CAD_cloud_msg.header.stamp = ros::Time::now();

  return 0;
}

void CADToPointCloud::setPCpath(const std::string& path)
{
  _pc_path = path;
}

// Following methods are extracted from Point Cloud Library (PCL)
  //                --> pcl/tools/mesh_sampling.cpp
  //
  //  Software License Agreement (BSD License)
  //
  //  Point Cloud Library (PCL) - www.pointclouds.org
  //  Copyright (c) 2010-2011, Willow Garage, Inc.
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //   * Redistributions of source code must retain the above copyright
  //     notice, this list of conditions and the following disclaimer.
  //   * Redistributions in binary form must reproduce the above
  //     copyright notice, this list of conditions and the following
  //     disclaimer in the documentation and/or other materials provided
  //     with the distribution.
  //   * Neither the name of the copyright holder(s) nor the names of its
  //     contributors may be used to endorse or promote products derived
  //     from this software without specific prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
  //  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  //  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  //  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
  //  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  //  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  //  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  //  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
  //  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  //  POSSIBILITY OF SUCH DAMAGE.
  //

void CADToPointCloud::uniform_sampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                                       PointCloudXYZ& cloud_out)
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

void CADToPointCloud::randPSurface(vtkPolyData* polydata, std::vector<double>* cumulativeAreas, double totalArea,
                                   Eigen::Vector4f& p)
{
  float r = static_cast<float>(uniform_deviate(rand()) * totalArea);

  std::vector<double>::iterator low = std::lower_bound(cumulativeAreas->begin(), cumulativeAreas->end(), r);
  vtkIdType el = vtkIdType(low - cumulativeAreas->begin());

  double A[3], B[3], C[3];
  vtkIdType npts = 0;
  vtkIdType* ptIds = NULL;
  polydata->GetCellPoints(el, npts, ptIds);
  polydata->GetPoint(ptIds[0], A);
  polydata->GetPoint(ptIds[1], B);
  polydata->GetPoint(ptIds[2], C);
  randomPointTriangle(float(A[0]), float(A[1]), float(A[2]), float(B[0]), float(B[1]), float(B[2]), float(C[0]),
                      float(C[1]), float(C[2]), p);
}

double CADToPointCloud::uniform_deviate(int seed)
{
  double ran = seed * (1.0 / (RAND_MAX + 1.0));
  return ran;
}

void CADToPointCloud::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1,
                                          float c2, float c3, Eigen::Vector4f& p)
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