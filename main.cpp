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

#include <igl/copyleft/cgal/delaunay_triangulation.h>
#include <igl/per_vertex_normals.h>
#include <igl/read_triangle_mesh.h>
#include <igl/cotmatrix.h>
#include <igl/grad.h>
#include <igl/doublearea.h>
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

  // Parse command line inputs
  std::string input_file;
  std::string output_file;
  std::string format;
  const char* yellowColor = "\033[33m";
  const char* resetColor = "\033[0m";

  for (int i = 1; i < argc; i++) {
      std::string arg(argv[i]);
      if (arg == "--in" && i + 1 < argc) {
          input_file = argv[++i];
      } else if (arg == "--out" && i + 1 < argc) {
          output_file = argv[++i];
      } else if (arg == "--format" && i + 1 < argc) {
          format = argv[++i];
          if (format != "mesh" && format != "point_cloud") {
              std::cerr << yellowColor << "Invalid format. Allowed formats: mesh, point_cloud"<< resetColor << std::endl;
              return 1;
          }
      } else {
          std::cerr << yellowColor << "Invalid argument: " << arg << resetColor << std::endl;
          return 1;
      }
  }

  if (input_file.empty() || output_file.empty() || format.empty()) {
        std::cerr << yellowColor << "Usage: " << argv[0] << " --in input.ply --out output.txt --format [mesh|point_cloud]" << resetColor << std::endl;
        return 1;
  }


  // Initialize polyscope
  polyscope::options::autocenterStructures = true;
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;
  polyscope::init();
  polyscope::options::groundPlaneMode = polyscope::GroundPlaneMode::ShadowOnly;

  // Read the mesh
  Eigen::MatrixXd V, N;
  Eigen::MatrixXi F;
  igl::read_triangle_mesh(input_file, V, F); 
  igl::per_vertex_normals(V,F,N);
  Eigen::VectorXd k_S(V.rows());
  
  // Calculate the total curvature
  if (format == "mesh"){
    TotalCurvatureMesh(V, F, N, k_S);
    VisTriangleMesh(k_S, V, F);
  }
  if (format == "point_cloud"){
    TotalCurvaturePointCloud(V, N, k_S, 20);
    VisPointCloud(k_S, V);
  }
  std::cout<<"finished calculating total curvature..."<<std::endl;

  // Save the result to txt file
  std::cout<<"writing results to txt file..."<<std::endl;
  std::ofstream ofs(output_file);
  if (!ofs.is_open()) {
      std::cerr << "Could not open the file for writing." << std::endl;
      return false;
  }
  ofs << k_S;
  ofs.close();
  std::cout<<"results saved to txt file."<<std::endl;
  
  // Show the gui
  polyscope::show();
  return 0;

}
