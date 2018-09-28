//
// Created by vasko on 25.07.18.
//

#ifndef BASE_OMPL_SPHERE_APROXIMATION_H
#define BASE_OMPL_SPHERE_APROXIMATION_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>


class Sphere_aproximation {
    void normalize_v3( Eigen::MatrixXd& arr);
    Eigen::MatrixXd divide_all(Eigen::MatrixXd& triangles_vertices ,
                               int new_triangle_count);
    void add_Mesh(const Eigen::MatrixXd& mesh_in_matrix);
    const Eigen::MatrixXd &get_mesh_matrix();
    Eigen::MatrixXd mesh_matrix;
    int num_of_iter=1;
    OpenMesh::TriMesh_ArrayKernelT<> meshOM;
    void Calculate();
    int num_of_spheres=0;
public:
    Sphere_aproximation(int number_of_iteration) :num_of_iter(number_of_iteration){
        Calculate() ;
    }
    const OpenMesh::TriMesh_ArrayKernelT<> &get_mesh();
    void Manipulate(double radius,::Eigen::Matrix<double, 3, 1> sphere_center);
    void addSphere(double radius,::Eigen::Matrix<double, 3, 1> sphere_center);
};


#endif //BASE_OMPL_SPHERE_APROXIMATION_H
