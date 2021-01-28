/**
 * @file CADToPointCloud.cpp
 * @copyright Copyright (c) 2020, FADA-CATEC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#include <CADToPointCloud.h>
#include <vtkTriangleFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkTriangle.h>

CADToPointCloud::CADToPointCloud(const std::string& cad_file_path, int sample_points)
{
    cad_file_path_ = cad_file_path;
    ROS_INFO("Converting file: %s", cad_file_path_.c_str());

    extension_ = boost::filesystem::extension(cad_file_path);
    ROS_INFO("File extension: %s", extension_.c_str());

    SAMPLE_POINTS_ = sample_points;
}

void CADToPointCloud::convertCloud(PointCloudRGB::Ptr cloud)
{
    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

    if(extension_==".ply")
    {
        pcl::PolygonMesh mesh;
        pcl::io::loadPolygonFilePLY(cad_file_path_, mesh);
        pcl::io::mesh2vtk(mesh, polydata);
    }
    else if(extension_==".obj")
    {
        vtkSmartPointer<vtkOBJReader> readerQuery = vtkSmartPointer<vtkOBJReader>::New();
        readerQuery->SetFileName(cad_file_path_.c_str());
        readerQuery->Update();
        polydata = readerQuery->GetOutput();
    }

    PointCloudRGB::Ptr CAD_cloud(new PointCloudRGB);
    uniformSampling(polydata, SAMPLE_POINTS_, *CAD_cloud);

    if(Utils::isValidCloud(CAD_cloud))
    {
        pcl::copyPointCloud(*CAD_cloud, *cloud);
    }
    else
        ROS_ERROR("Error converting CAD mesh to pointcloud");
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

void CADToPointCloud::uniformSampling(vtkSmartPointer<vtkPolyData> polydata, size_t n_samples,
                                       PointCloudRGB& cloud_out)
{
    //make sure that the polygons are triangles
    vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
    triangleFilter->SetInputData(polydata);
    triangleFilter->Update();

    vtkSmartPointer<vtkPolyDataMapper> triangleMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    triangleMapper->SetInputConnection(triangleFilter->GetOutputPort());
    triangleMapper->Update();
    polydata = triangleMapper->GetInput();

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
    float r = static_cast<float>(uniformDeviate(rand()) * totalArea);

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

double CADToPointCloud::uniformDeviate(int seed)
{
    double ran = seed * (1.0 / (RAND_MAX + 1.0));
    return ran;
}

void CADToPointCloud::randomPointTriangle(float a1, float a2, float a3, float b1, float b2, float b3, float c1,
                                          float c2, float c3, Eigen::Vector4f& p)
{
    float r1 = static_cast<float>(uniformDeviate(rand()));
    float r2 = static_cast<float>(uniformDeviate(rand()));
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
