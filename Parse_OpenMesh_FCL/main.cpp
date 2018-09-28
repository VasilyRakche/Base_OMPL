
#include "get_OpenMesh_data.h"
//Example of usage for the getting the data from some directory, in this example from
// /home/vasko/CLionProjects/Open_mesh_getData/cmake-build-debug/BugTrap_planar_env.ply
int main()
{
    typedef fcl::BVHModel<fcl::OBBRSS> Model;
    std::shared_ptr<Model> model;
    try {
        model = get_OpenMesh_data::model(
                "/home/vasko/CLionProjects/Open_mesh_getData/cmake-build-debug/BugTrap_planar_env.ply");
    }catch (std::exception& e ) {
        std::cout << "Error " << e.what() << std::endl;
    }
    return 0;
}
