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

#include "polyscope/polyscope.h"
#include "polyscope/point_cloud.h"
#include "polyscope/surface_mesh.h"



void VisTriangleMesh(Eigen::VectorXd k_S, Eigen::MatrixXd V, Eigen::MatrixXi F){
  Eigen::VectorXd k_S_vis(V.rows());
  k_S_vis = k_S.array().pow(0.0425);
  auto psMesh = polyscope::registerSurfaceMesh("my mesh Laplacian", V, F);
  auto TotalCurvature = polyscope::getSurfaceMesh("my mesh Laplacian");
  auto ScalarQuantity1 = TotalCurvature->addVertexScalarQuantity("TotalCurvature", k_S_vis);
  ScalarQuantity1->setEnabled(true);       
  ScalarQuantity1->setColorMap("jet");   
}

void VisPointCloud(Eigen::VectorXd k_S, Eigen::MatrixXd V){
  Eigen::VectorXd k_S_vis(V.rows());
  k_S_vis = k_S.array().pow(0.0625);
  auto psCloud = polyscope::registerPointCloud("my pcd", V);
  auto TotalCurvature = polyscope::getPointCloud("my pcd")->addScalarQuantity("TotalCurvature", k_S_vis);
  TotalCurvature->setColorMap("jet");  
}