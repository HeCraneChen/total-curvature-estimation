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



double PerTriangleLaplacianCurvatureFast(int row_id, const Eigen::MatrixXd& V, const Eigen::MatrixXd& N, const Eigen::MatrixXi& F, const Eigen::MatrixXd& A)
{  
    Eigen::MatrixXd n_adjacent_face, v_adjacent_face, x, y, z;
    Eigen::MatrixXi f(1, 3), adjacent_faces, adjacent_face;
    Eigen::SparseMatrix<double> l;
    double total_curvature_one_triangle, cx, cy, cz, face_area;
    face_area = *A(row_id, Eigen::placeholders::all).data();
    face_area = face_area / 2;

    adjacent_face = F(row_id, Eigen::placeholders::all);
    v_adjacent_face = V({adjacent_face(0), adjacent_face(1), adjacent_face(2)}, Eigen::placeholders::all);
    n_adjacent_face = N({adjacent_face(0), adjacent_face(1), adjacent_face(2)}, Eigen::placeholders::all);
    
    f << 0 , 1 , 2;
    igl::cotmatrix(v_adjacent_face, f, l);
    x = n_adjacent_face(Eigen::placeholders::all,0);
    y = n_adjacent_face(Eigen::placeholders::all,1);
    z = n_adjacent_face(Eigen::placeholders::all,2);
    cx = (x.transpose() * l * x)(0);
    cy = (y.transpose() * l * y)(0);
    cz = (z.transpose() * l * z)(0);
    total_curvature_one_triangle = - cx - cy - cz;

    return total_curvature_one_triangle;
}



void TotalCurvatureMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Eigen::VectorXd& k_S)
{
  using namespace Eigen;
  using namespace std;

  Eigen::SparseMatrix<double> G;
  Eigen::MatrixXd N, A;
  igl::per_vertex_normals(V,F,N);
  igl::doublearea(V, F, A);
  igl::grad(V, F, G);
  int V_num = V.rows();
  int F_num = F.rows();
  Eigen::VectorXd k_S_face(F_num);

  // per-face curvature
  #pragma omp parallel for
  for(int i = 0; i<F_num; i++){
    k_S_face(i) = PerTriangleLaplacianCurvatureFast(i, V, N, F, A);
  } 
  // deriving per-vertex curvature
  #pragma omp parallel for
  for (int i = 0; i<V_num; i++){
    double total_area = 0;
    double k_S_entry = 0;
    int N_adjacent_faces = 0;
    for (SparseMatrix<double>::InnerIterator it(G,i); it; ++it){
      if (it.row() < F.rows()){ 
        total_area += A(it.row());
        k_S_entry += k_S_face(it.row()); 
      }
    }
    k_S(i) = k_S_entry / total_area;
  }
}