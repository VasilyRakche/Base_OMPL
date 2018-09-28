//
// Created by vasko on 20.07.18.
//

#ifndef BASE_OMPL_ISVALID_CLASS_H
#define BASE_OMPL_ISVALID_CLASS_H
#include <ompl/base/StateValidityChecker.h>
#include <fcl/BVH/BVH_model.h>
#include "FCL_STATE_CHECK/FCL_StateInspection.h"
class ValidityChecker : public ompl::base::StateValidityChecker
{
public:
    ValidityChecker(const std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> model,std::shared_ptr<ompl::base::SpaceInformation> si)
            :StateValidityChecker(si), model_FCL(model){
    }
    bool isValid(const ompl::base::State* state) const override ;

private:
    std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> model_FCL;
};

inline bool ValidityChecker::isValid(const ompl::base::State *state) const {
    return FCL_StateChecker::isValid(state,model_FCL);
}
#endif //BASE_OMPL_ISVALID_CLASS_H
