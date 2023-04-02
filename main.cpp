// Copyright (C) 2023 Crane Chen <cranechen7@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <omp.h>
#include <thread>
#include <sstream>
#include <random>
#include <vector>
#include <algorithm>
#include <utility>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/copyleft/cgal/delaunay_triangulation.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/doublearea.h>
#include <igl/principal_curvature.h>
#include <igl/knn.h>
#include <igl/octree.h>

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"

#include "total_curvature_mesh.cpp"
#include "total_curvature_point_cloud.cpp"
#include "visualization.cpp"

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;
  using namespace std::chrono;
  Eigen::MatrixXd V;
  Eigen::MatrixXi F;

  // Initialize polyscope
  polyscope::options::autocenterStructures = true;
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;
  polyscope::init();
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

  // Read the mesh
  igl::read_triangle_mesh("../example_data/cow.ply", V, F);
  // igl::read_triangle_mesh("../example_data/torus_res36.ply", V, F);
  Eigen::VectorXd k_S(V.rows());
  
  // //////////////// Calculate the curvature of mesh
  // TotalCurvatureMesh(V, F, k_S);
  // VisTriangleMesh(k_S, V, F);

  // // //////////////// Calculate the curvature of point cloud
  Eigen::MatrixXd N;
  igl::per_vertex_normals(V,F,N);
  TotalCurvaturePointCloud(V, N, k_S, 20);
  VisPointCloud(k_S, V);

  // Show the gui
  polyscope::show();
  return 0;

}
