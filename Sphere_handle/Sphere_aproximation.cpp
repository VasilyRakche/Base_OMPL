//
// Created by vasko on 25.07.18.
//

#include "Sphere_aproximation.h"

using namespace Eigen;
using namespace std;
typedef OpenMesh::TriMesh_ArrayKernelT<> MyMesh;
void Sphere_aproximation::normalize_v3( MatrixXd& arr){
    // Normalize a numpy array of 3 component vectors shape=(n,3)
    double lens=0;
    for(int i=0;i<arr.rows();i++) {
        lens = sqrt(arr(i,0)*arr(i,0) +arr(i,1)*arr(i,1)
                    +arr(i,2)*arr(i,2));
        arr(i,0)/=lens;
        arr(i,1)/=lens;
        arr(i,2)/=lens;
    }
}
MatrixXd Sphere_aproximation::divide_all(MatrixXd& triangles_vertices , int new_triangle_count){
    MatrixXd new_triangle_vertices(new_triangle_count,triangles_vertices.cols());
    //# Subdivide each triangle in the old approximation and normalize
    //#  the new points thus generated to lie on the surface of the unit
    //#  sphere.
    //# Each input triangle with vertices labelled [0,1,2] as shown
    //#  below will be turned into four new triangles:
    //#
    //    #            Make new points
    //#                 a = (0+2)/2
    //#                 b = (0+1)/2
    //#                 c = (1+2)/2
    //#        1
    //#       /\        Normalize a, b, c
    //#      /  \
    //#    b/____\ c    Construct new triangles
    //#    /\    /\       t1 [0,b,a]
    //#   /  \  /  \      t2 [b,1,c]
    //#  /____\/____\     t3 [a,b,c]
    //# 0      a     2    t4 [a,c,2]

    //Making new points:
    MatrixXd    v0(triangles_vertices.rows(),triangles_vertices.cols()/3),
            v1(triangles_vertices.rows(),triangles_vertices.cols()/3),
            v2(triangles_vertices.rows(),triangles_vertices.cols()/3);
    for(int i=0;i<triangles_vertices.rows();i++) {
        for(int j=0;j<triangles_vertices.cols()/3;j++) {
            v0(i, j) = triangles_vertices(i, j);
            v1(i, j) = triangles_vertices(i, 3+j);
            v2(i, j) = triangles_vertices(i, 6+j);
        }
    }

    MatrixXd    a(triangles_vertices.rows(),triangles_vertices.cols()/3),
            b(triangles_vertices.rows(),triangles_vertices.cols()/3),
            c(triangles_vertices.rows(),triangles_vertices.cols()/3);
    a = ( v0+v2 ) * 0.5;
    b = ( v0+v1 ) * 0.5;
    c = ( v1+v2 ) * 0.5;

    normalize_v3( a );
    normalize_v3( b );
    normalize_v3( c );

    //Constructing the triangles:
    //#        1
    //#       /\
    //#      /  \
    //#    b/____\ c    Construct new triangles
    //#    /\    /\       t1 [0,b,a]
    //#   /  \  /  \      t2 [b,1,c]
    //#  /____\/____\     t3 [a,b,c]
    //# 0      a     2    t4 [a,c,2]
    for(int i=0;i<new_triangle_count;i++) {
        for(int j=0;j<triangles_vertices.cols()/3;j++) {

            if(i < triangles_vertices.rows()){
                new_triangle_vertices(i, j) = v0(i, j);
                new_triangle_vertices(i, j + 3) = b(i, j);
                new_triangle_vertices(i, j + 6) = a(i, j);
            }else if((triangles_vertices.rows() <= i)&(i < triangles_vertices.rows() * 2)){
                new_triangle_vertices(i, j) = b(i-triangles_vertices.rows(), j);
                new_triangle_vertices(i, j + 3) = v1(i-triangles_vertices.rows(), j);
                new_triangle_vertices(i, j + 6) = c(i-triangles_vertices.rows(), j);
            }else if((triangles_vertices.rows()*2 <= i)&(i < triangles_vertices.rows() * 3)){
                new_triangle_vertices(i, j) = a(i-triangles_vertices.rows()*2, j);
                new_triangle_vertices(i, j + 3) = b(i-triangles_vertices.rows()*2, j);
                new_triangle_vertices(i, j + 6) = c(i-triangles_vertices.rows()*2, j);
            }else if((triangles_vertices.rows()*3 <= i)&(i <triangles_vertices.rows() * 4)){
                new_triangle_vertices(i, j) = a(i-triangles_vertices.rows()*3, j);
                new_triangle_vertices(i, j + 3) = c(i-triangles_vertices.rows()*3, j);
                new_triangle_vertices(i, j + 6) = v2(i-triangles_vertices.rows()*3, j);
            }else
                throw runtime_error("It went out of the loop, while generating 'new_triangle_vertices'");

        }
    }
    triangles_vertices.resize(new_triangle_count,triangles_vertices.cols());
    triangles_vertices=new_triangle_vertices;

    return new_triangle_vertices;
}

void Sphere_aproximation::Calculate(){
    if (num_of_iter<0) throw runtime_error("n should be bigger then 0");
    //In matrix(row,collumns)
    MatrixXd octahedron_vertices(6,3);

    octahedron_vertices
            <<    1,  0,  0,  //0
            -1,  0,  0,  //1
            0,  1,  0,  //2
            0, -1,  0,  //3
            0,  0,  1,  //4
            0,  0, -1;  //5

    MatrixXd octahedron_triangles(8,3);

    octahedron_triangles
            <<      0,  4,  2,
            2,  4,  1,
            1,  4,  3,
            3,  4,  0,
            0,  2,  5,
            2,  1,  5,
            1,  3,  5,
            3,  0,  5;
    //In matrix matrix_with_triangles data will be stored this way
    //Each row will represent the triangle
    //#        1
    //#       /\        Normalize a, b, c
    //#      /  \
    //#    0/_ __\2     Construct new triangles
    //#                 The data will be stored this way:
    //#                 (x0,y0,z0,x1,y1,z1,x2,y2,z2)

    MatrixXd    matrix_with_triangles(octahedron_triangles.rows(),octahedron_vertices.cols()*3);
    for(int i=0;i<octahedron_triangles.rows();i++) {
        for(int j=0;j<octahedron_triangles.cols();j++) {
            for(int z=0;z<octahedron_vertices.cols();z++) {
                matrix_with_triangles(i, 3*j+z)=octahedron_vertices(octahedron_triangles(i,j),z);
            }
        }
    }
    

    //Set the new value for number of triangles
    int new_triangle_count= matrix_with_triangles.rows();
    MatrixXd devide(octahedron_triangles.rows(),octahedron_vertices.cols()*3);
    devide=matrix_with_triangles;

    for(int i=0;i<num_of_iter;i++){
        new_triangle_count*=4; //new number of triangles
        devide.resize(new_triangle_count,matrix_with_triangles.cols()); //resize so it can fit new data
        devide=divide_all(matrix_with_triangles,new_triangle_count); //get new data

    }
    mesh_matrix.resize(new_triangle_count,matrix_with_triangles.cols());
    mesh_matrix=devide;

}
void Sphere_aproximation::add_Mesh(const Eigen::MatrixXd& mesh_in_matrix){
    MyMesh::VertexHandle vhandle[3];
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    for(int i=0;i<mesh_in_matrix.rows();i++) {
        face_vhandles.clear();
        for(int j=2;j>-1;j--) {
            if(num_of_spheres==1) {
                vhandle[j] = meshOM.add_vertex(MyMesh::Point(mesh_in_matrix(i, 3 * (2-j) ),      //Its clockwise
                                                             mesh_in_matrix(i, 3 * (2-j) + 1),
                                                             mesh_in_matrix(i, 3 * (2-j) + 2)));
            }else {
                vhandle[j] = meshOM.add_vertex(MyMesh::Point(mesh_in_matrix(i, 3 * j),        //Its counter clockwise
                                                             mesh_in_matrix(i, 3 * j + 1),
                                                             mesh_in_matrix(i, 3 * j + 2)));
            }
            face_vhandles.push_back(vhandle[j]);
            
        }
        meshOM.add_face(face_vhandles);
    }
}
void Sphere_aproximation::Manipulate(double radius,::Eigen::Matrix<double, 3, 1> sphere_center){
    if(mesh_matrix.size()<2) throw runtime_error("You dont have a Sphere calculated");
    mesh_matrix*=radius;
    for(int i=0;i<mesh_matrix.rows();i++) {
        for(int j=0;j<3;j++) {
            mesh_matrix(i,3*j)+=sphere_center(0,0);
            mesh_matrix(i,3*j+1)+=sphere_center(1,0);
            mesh_matrix(i,3*j+2)+=sphere_center(2,0);
        }
    }
//    std::cout<<"Here is mesh_matrix:\n"<<mesh_matrix<<endl; //Print the sphere matrix

}
const Eigen::MatrixXd &Sphere_aproximation::get_mesh_matrix(){
    return mesh_matrix;
}
const OpenMesh::TriMesh_ArrayKernelT<> &Sphere_aproximation::get_mesh(){
    return meshOM;
}

void Sphere_aproximation::addSphere(double radius,::Eigen::Matrix<double, 3, 1> sphere_center){
    Sphere_aproximation Sphere_to_add(*this);
    //Sphere_to_add=*this;
    this->num_of_spheres++;
    std::cout<<"Here is coordinates of "<<num_of_spheres<<"-st sphere center:\n"<<sphere_center(0,0)<<endl
                                                                             <<sphere_center(1,0)<<endl
                                                                             <<sphere_center(2,0)<<endl;
    std::cout<<"Here is radius of "<<num_of_spheres<<"-st sphere:\n"<<radius<<endl;
    Sphere_to_add.Manipulate(radius,sphere_center);
    this->add_Mesh(Sphere_to_add.get_mesh_matrix());
}
