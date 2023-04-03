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



std::vector<int> where(int i, Eigen::MatrixXi inArray)
{
    std::vector<int> res;
    int idx = 0;
    for (int r = 0; r < inArray.rows(); r++){
      for (int c = 0; c < inArray.cols(); c++){
        if(inArray(r,c) == i){
          res.push_back(r);
        }
      }
    }
    return res;
}



Eigen::MatrixXi Delaunay_KNN(Eigen::MatrixXd knn_locations_including_self, Eigen::MatrixXi idx){   
    Eigen::MatrixXi adjacent_triangles_idx_local, adjacent_triangles_idx, faces, all_triangles;
    Eigen::MatrixXd n, A;
    Eigen::Vector3d mean_A_vec, r0, x_axis, r1, n_plane;

    // projecting points from 3d to 2d, find the best fitting plane
    mean_A_vec = knn_locations_including_self.transpose().rowwise().mean();
    A = knn_locations_including_self.transpose().colwise() - mean_A_vec; // 3, 20 
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    n = svd.matrixU()(Eigen::placeholders::all, svd.matrixU().cols()-1);
    n_plane = {n(0), n(1), n(2)};
    r0 = {knn_locations_including_self(0,0), knn_locations_including_self(0,1), knn_locations_including_self(0,2)};
    x_axis = {1, 0, 0};
    auto e1 = n_plane.cross(x_axis);
    auto e2 = n_plane.cross(e1);
    auto tmp = knn_locations_including_self.rowwise() - r0.transpose();
    auto t1 = tmp*e1; 
    auto t2 = tmp*e2;

    // Delaunay triangulation to find one ring idx
    Eigen::MatrixXd knn_locations_2d(t1.rows(), 2);
    knn_locations_2d << t1, t2; // 20, 2
    igl::copyleft::cgal::delaunay_triangulation(knn_locations_2d, all_triangles);
    std::vector<int> adjacent_triangles_mask = where(0, all_triangles);
    adjacent_triangles_idx_local =  all_triangles(adjacent_triangles_mask, Eigen::placeholders::all); // 5, 3

    // Converting indices back to original global indices
    Eigen::MatrixXi adjacent_triangles_idx_x = idx(Eigen::placeholders::all,adjacent_triangles_idx_local(Eigen::placeholders::all,0));
    Eigen::MatrixXi adjacent_triangles_idx_y = idx(Eigen::placeholders::all,adjacent_triangles_idx_local(Eigen::placeholders::all,1));
    Eigen::MatrixXi adjacent_triangles_idx_z = idx(Eigen::placeholders::all,adjacent_triangles_idx_local(Eigen::placeholders::all,2));
    Eigen::MatrixXi adjacent_triangles_idx_tmp(3, adjacent_triangles_idx_x.cols());
    adjacent_triangles_idx_tmp << adjacent_triangles_idx_x, adjacent_triangles_idx_y, adjacent_triangles_idx_z;
    adjacent_triangles_idx = adjacent_triangles_idx_tmp.transpose();
    return adjacent_triangles_idx;
}



double PerTriangleLaplacianTriangleFanCurvature(Eigen::MatrixXi adjacent_triangles_idx, const Eigen::MatrixXd& V, const Eigen::MatrixXd& N){
    Eigen::MatrixXd triangle, n_triangle_face, v_triangle_fan, n_triangle_fan, x, y, z;
    Eigen::MatrixXi f(1, 3), adjacent_triangle_idx;
    Eigen::SparseMatrix<double> l;
    double cx, cy, cz;
    double total_curvature = 0.0, total_area = 0.0;
    int N_triangles = adjacent_triangles_idx.rows();
    for (int i = 0; i < N_triangles; i++){    
      adjacent_triangle_idx = adjacent_triangles_idx(i,Eigen::placeholders::all);
      std::vector<int> triangle_verts = {adjacent_triangle_idx(0,0), adjacent_triangle_idx(0,1), adjacent_triangle_idx(0,2)};
      triangle = V(triangle_verts, Eigen::placeholders::all); //3, 3
      n_triangle_face = N(triangle_verts, Eigen::placeholders::all); // 3, 3
      Eigen::MatrixXd AB_mat = triangle(0, Eigen::placeholders::all) - triangle(1, Eigen::placeholders::all);
      Eigen::MatrixXd AC_mat = triangle(0, Eigen::placeholders::all) - triangle(2, Eigen::placeholders::all);
      Eigen::Vector3d AB(AB_mat.coeff(0, 0), AB_mat.coeff(0, 1), AB_mat.coeff(0, 2));
      Eigen::Vector3d AC(AC_mat.coeff(0, 0), AC_mat.coeff(0, 1), AC_mat.coeff(0, 2));
      double triangle_area = 0.5 * sqrt((AB.cross(AC)).squaredNorm());
      f << 0 , 1 , 2;
      igl::cotmatrix(triangle, f, l);
      x = n_triangle_face(Eigen::placeholders::all,0);
      y = n_triangle_face(Eigen::placeholders::all,1);
      z = n_triangle_face(Eigen::placeholders::all,2);
      cx = (x.transpose() * l * x)(0);
      cy = (y.transpose() * l * y)(0);
      cz = (z.transpose() * l * z)(0); 
      total_curvature += (- cx - cy - cz);
      total_area += triangle_area;
    }
    total_curvature = total_curvature / total_area;
    return total_curvature;
}



bool HasDuplicateVertices(const Eigen::MatrixXd& V, double threshold = 1e-6) {
    const char* blueColor = "\033[34m";
    const char* resetColor = "\033[0m";
    for (int i = 0; i < V.rows(); ++i) {
        for (int j = i + 1; j < V.rows(); ++j) {

            // Compute the Euclidean distance between vertices i and j
            double distance = (V.row(i) - V.row(j)).norm();     

            // Check if the distance is smaller than the threshold
            if (distance < threshold) {
                std::cerr << blueColor << "Oops, your data didn't pass sanity check. Duplicate vertices found. You can remove the dupicate vertices with MeshLab (Filter -> Cleaning and Repairing -> Remove Duplicate Vertices) and then try again. Bad data can result in segmentation fault when building the octree.)" << resetColor << std::endl;
                return true;
            }
        }
    }
    return false;
}



void TotalCurvaturePointCloud(const Eigen::MatrixXd& V, const Eigen::MatrixXd& N, Eigen::VectorXd& k_S, int K = 20){
  // initializing variables
  std::vector<std::vector<int > > O_PI;
  Eigen::MatrixXi O_CH;
  Eigen::MatrixXd O_CN;
  Eigen::VectorXd O_W;
  Eigen::MatrixXi I;

  // build octree
  // std::cout<<"data sanity check..."<<std::endl;
  // bool sanity_check = HasDuplicateVertices(V);
  // std::cout<<"data passed sanity check!"<<std::endl;
  std::cout<<"building octree..."<<std::endl;
  igl::octree(V,O_PI,O_CH,O_CN,O_W);
  std::cout<<"successfully built octree!"<<std::endl;

  // KNN
  std::cout<<"calculating knn..."<<std::endl;
  igl::knn(V,K,O_PI,O_CH,O_CN,O_W,I);

  // Curvature Estimation
  std::cout<<"calculating total curvature..."<<std::endl;
  #pragma omp parallel for
  for (int i =0; i<V.rows(); i++){
    std::vector<int> idx_vec;
    Eigen::MatrixXi idx;
    idx = I(i,Eigen::placeholders::all);
    for (int j = 0; j<K; j++){
      idx_vec.push_back(idx(j));
    }
    Eigen::MatrixXd knn_locations_including_self = V(idx_vec,Eigen::placeholders::all);
    Eigen::MatrixXi adjacent_triangles_idx = Delaunay_KNN(knn_locations_including_self, idx);
    k_S(i) = PerTriangleLaplacianTriangleFanCurvature(adjacent_triangles_idx, V, N);
  }
}

