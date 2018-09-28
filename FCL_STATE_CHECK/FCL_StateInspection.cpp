//
// Created by vasko on 23.07.18.
//

#include "FCL_StateInspection.h"
#include <fcl/broadphase/broadphase_SSaP.h>
//#include "../../CLionProjects/Parse_OpenMesh>FCL/get_OpenMesh_data.h"
namespace ob=ompl::base;
bool FCL_StateChecker::isValid(const ob::State *state,const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &model)  {
    //Checking validity with the sphere as one object and model as another
    // cast the abstract state type to the type we expect
    const auto *se3state = state->as<ob::SE3StateSpace::StateType>();

    // extract the first component of the state and cast it to what we expect
    const auto *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

    //Vector for holding the position of the state
    fcl::Vec3f vec3f_Sphere;
    vec3f_Sphere.setValue((*pos)[0],(*pos)[1],(*pos)[2]);
    fcl::Matrix3f rotSphere(1.0, 0.0, 0.0,
                           0.0, 1.0, 0.0,
                           0.0, 0.0, 1.0);
    std::shared_ptr< fcl::CollisionGeometry > cgeomSphere_ = std::make_shared<fcl::Sphere>(0.1);
    auto objSphere = fcl::CollisionObject(cgeomSphere_, rotSphere, vec3f_Sphere);
    //For determening another object for collision inspection
    //    Making a Collision object from model
    fcl::CollisionObject* obj1 = new fcl::CollisionObject(model);
    // set the distance request structure, here we just use the default setting
    fcl::CollisionRequest request;
    // result will be returned via the collision result structure
    fcl::CollisionResult result;
    // perform distance test
    auto collide = fcl::collide(&objSphere, obj1, request, result);
    std::cout << "collide " << collide << std::endl;

    return !result.isCollision();
}

double FCL_DistanceToCollision::Callculate(const Vector3Ptr state,
                                        const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &model) {
    //When we calculate distance we represent a given object as a simplest object dot

    //Vector for holding the position of the state
    fcl::Vec3f vec3f_Sphere;
    vec3f_Sphere.setValue((*state)[0],(*state)[1],(*state)[2]);
    fcl::Matrix3f rotSphere(1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0);
    std::shared_ptr< fcl::CollisionGeometry > cgeomSphere_ = std::make_shared<fcl::Sphere>(0.1);
    auto objSphere = fcl::CollisionObject(cgeomSphere_, rotSphere, vec3f_Sphere);

    //Part when i tried to make dummy collision object ***************************
    /*//Making the AABB structure, its a box in 3D determened by the two diagonal points
    auto *aabb=new fcl::AABB(vec3f);
    //Making a simplest Collision object(a point)
    fcl::CollisionObject *dum_obj= new DummyCollisionObject(*aabb);*/
    //******************************************************************

    //For determening another object for collision inspection
    //    Making a Collision object from model
    fcl::CollisionObject* obj1 = new fcl::CollisionObject(model);
    // set the distance request structure, here we just use the default setting
//    fcl::DistanceRequest request(fcl::GST_INDEP);
    fcl::DistanceRequest request;
    // result will be returned via the collision result structure
    fcl::DistanceResult result;
    // perform distance test
    auto distance=fcl::distance(&objSphere,obj1,request,result);
    std::cout << "collide " << distance << std::endl;
    return result.min_distance;

}