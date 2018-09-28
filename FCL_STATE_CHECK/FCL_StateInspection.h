//
// Created by vasko on 23.07.18.
//

#ifndef BASE_OMPL_FCL_STATEINSPECTION_H
#define BASE_OMPL_FCL_STATEINSPECTION_H

#include <ompl/base/spaces/SE3StateSpace.h>

#include <fcl/BVH/BVH_model.h>
#include <fcl/collision_data.h>
#include <fcl/collision.h>
#include <fcl/distance.h>
#include <fcl/BV/AABB.h>
#include <fcl/collision_object.h>
#include <eigen3/Eigen/Core>

class FCL_StateChecker {
public:
    static bool isValid(const ompl::base::State *state, const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &model)  ;
};

class DummyCollisionObject : public fcl::CollisionObject //Dummy collision object, not used
{
public:
    DummyCollisionObject(const fcl::AABB& aabb_) : CollisionObject(std::shared_ptr<fcl::CollisionGeometry>())
    {
        aabb = aabb_;
    }
};
class FCL_DistanceToCollision{
typedef ::std::shared_ptr< Eigen::Matrix<double, 3, 1>> Vector3Ptr;
public:
    static double Callculate(const Vector3Ptr state, const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> &model);
};

#endif //BASE_OMPL_FCL_STATEINSPECTION_H
