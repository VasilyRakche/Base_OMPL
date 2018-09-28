//
// Created by vasko on 23.07.18.
//
// Copyright (c) 2009, Markus Rickert
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
//http://docs.ros.org/fuerte/api/fcl/html/classfcl_1_1DummyCollisionObject.html
#ifndef BASE_OMPL_WORKSPACESPHERE_H
#define BASE_OMPL_WORKSPACESPHERE_H

#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <eigen3/Eigen/Core>
#include "Sphere_aproximation.h"

typedef ::Eigen::Matrix<double, 3, 1> Vector3;
typedef ::std::shared_ptr< Vector3> Vector3Ptr;

namespace rl
{
    namespace plan
    {
        struct WorkspaceSphere
        {
            typedef ::boost::adjacency_list_traits<
                    ::boost::listS,
                    ::boost::listS,
                    ::boost::bidirectionalS
            >::vertex_descriptor Vertex;

            bool operator<(const WorkspaceSphere& rhs) const;

            Vector3Ptr center;

            Vertex parent;

            double priority;

            double radius;

            double radiusSum;
        };
    }
}


#endif //BASE_OMPL_WORKSPACESPHERE_H
