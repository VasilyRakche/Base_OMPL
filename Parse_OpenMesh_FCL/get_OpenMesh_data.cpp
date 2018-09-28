//
// Created by vasko on 18.07.18.
//

#include "get_OpenMesh_data.h"

//using fcl::BVHModel<fcl::OBBRSS> as a type for our Model
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

void get_OpenMesh_data::add_boundaries_for_bagtrap(OpenMesh::TriMesh_ArrayKernelT<> &mesh){
    //NOW WE NEED TO ADD ADDITIONAL BOUNDARIES FOR PARTICULAR MODEL
    Eigen::MatrixXd verticesXd(8,3);
    verticesXd
            <<    55,        0,     -55.01,  //0
            55,        0,      55.01,  //1
            -55,        0,      55.01,  //2
            -55,        0,     -55.01,  //3
            55,    -8.17,     -55.01,  //4
            55,    -8.17,      55.01,  //5
            -55,    -8.17,      55.01,  //6
            -55,    -8.17,     -55.01;  //7

    Eigen::MatrixXd trianglesXd(4,3);

    trianglesXd
            <<      2,  1,  0,
            2,  0,  3,
            6,  4,  5,
            6,  7,  4;
    Eigen::MatrixXd    matrix_with_triangles(trianglesXd.rows(),verticesXd.cols()*3);
    for(int i=0;i<trianglesXd.rows();i++) {
        for(int j=0;j<trianglesXd.cols();j++) {
            for(int z=0;z<verticesXd.cols();z++) {
                matrix_with_triangles(i, 3*j+z)=verticesXd(trianglesXd(i,j),z);
            }
        }
    }

    MyMesh::VertexHandle vhandle[3];
    std::vector<MyMesh::VertexHandle>  face_vhandles;
    for(int i=0;i<matrix_with_triangles.rows();i++) {
        face_vhandles.clear();
        for(int j=0;j<3;j++) {
            vhandle[j] = mesh.add_vertex(MyMesh::Point(matrix_with_triangles(i, 3 * j),
                                                       matrix_with_triangles(i, 3 * j + 1),
                                                       matrix_with_triangles(i, 3 * j + 2)));
            face_vhandles.push_back(vhandle[j]);

        }
        mesh.add_face(face_vhandles);
    }
}

std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> get_OpenMesh_data::model(std::string file_name, bool add_bagtrap_bounds){
    //For OpenMesh
    MyMesh mesh;
    MyMesh::Point point_it;
    double x=0,y=0,z=0;
    MyMesh::FaceVertexIter fv_it;

    //For FCL
    typedef fcl::BVHModel<fcl::OBBRSS> Model;
    auto model = std::make_shared<Model>();
    std::vector<fcl::Vec3f>  vertices;
    fcl::Vec3f vec3f;

    //Read my dae file to mesh
    if ( !OpenMesh::IO::read_mesh(mesh,file_name) )
    {
        std::cerr << "Cannot read mesh from file, hint 'put the absolute path or try another format( i.e. .ply )'" << std::endl;
        throw std::runtime_error("There was an exception, couldnt read the file");
//        return nullptr; //NOT SURE HOW TO HANDLE THIS SITUATION
    }
    if(add_bagtrap_bounds) {
        get_OpenMesh_data::add_boundaries_for_bagtrap(mesh);
    }
    std::cout<<"Boundaries are added to a mesh"<<std::endl;
    //ADDITIONAL SETTINGS FOR THE MODEL

    model->beginModel(); //Start generating the model
    // iterator over all faces
    for (MyMesh::FaceIter f_it=mesh.faces_begin(); f_it!=mesh.faces_end(); ++f_it) {
        //iterate over all vertexes in face
        for (fv_it = mesh.fv_iter(*f_it); fv_it.is_valid(); ++fv_it) {
            //get coordinates
            point_it = mesh.point(fv_it);
            x = point_it[0];
            y = point_it[1];
            z = point_it[2];
            // generate vertices for FCL
            vec3f.setValue(x, y, z);
            //Add vertice to the vec for generating triangles FCL
            vertices.push_back(vec3f);
        }
        //Add triangle to the model
        model->addTriangle(vertices[0],vertices[1],vertices[2]);
        //Clear the vector with coordinates for triangle vertices
        vertices.clear();
    }

    //PRINT THE MODEL

    model->endModel(); //Finish generating the model
    std::cout<<"Model generated"<<std::endl;
    if ( !OpenMesh::IO::write_mesh(mesh, "Model_with_boundries.ply") )
    {
        std::cerr << "Cannot write mesh to file 'output.ply'" << std::endl;
    }
    return model;

}