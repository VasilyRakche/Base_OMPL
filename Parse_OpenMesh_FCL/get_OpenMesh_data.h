//
// Created by vasko on 18.07.18.
//

#ifndef PARSE_OPENMESH_FCL_GET_OPENMESH_DATA_H
#define PARSE_OPENMESH_FCL_GET_OPENMESH_DATA_H
// -------------------- FCL
#include "fcl/shape/geometric_shapes.h"
#include <fcl/BVH/BVH_model.h>
// --------------------end FCL

// -------------------- OpenMesh
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/IO/Options.hh>
// --------------------end 
#include <eigen3/Eigen/Dense>

class get_OpenMesh_data {
public:
    static std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> model(std::string file_name, bool add_bagtrap_bounds);
    static void add_boundaries_for_bagtrap(OpenMesh::TriMesh_ArrayKernelT<> &mesh);
};


#endif //PARSE_OPENMESH_FCL_GET_OPENMESH_DATA_H
