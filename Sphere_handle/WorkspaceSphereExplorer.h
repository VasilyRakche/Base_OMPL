//
// Created by vasko on 23.07.18.
//
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

#ifndef BASE_OMPL_WORKSPACESPHEREEXPLORER_H
#define BASE_OMPL_WORKSPACESPHEREEXPLORER_H


#include <list>
#include <random>
#include <set>
#include <eigen3/Eigen/Core>

#include <boost/graph/adjacency_list.hpp>
#include <boost/random/uniform_on_sphere.hpp>
#include <boost/random/variate_generator.hpp>
//#include <rl/math/Vector.h>

#include "WorkspaceSphere.h"
#include "WorkspaceSphereList.h"

//Upgrade from original, include FCL model
#include <fcl/BVH/BVH_model.h>
#include "../FCL_STATE_CHECK/FCL_StateInspection.h" //To include distance computation with FCL

namespace rl
{
    namespace plan
    {
        typedef ::Eigen::Matrix<double, 3, 1> Vector3;
//        class DistanceModel;

        /**
         * Wavefront expansion.
         *
         * Oliver Brock and Lydia E. Kavraki. Decomposition-based motion planning:
         * A framework for real-time motion planning in high-dimensional configuration
         * spaces. In Proceedings of the IEEE International Conference on Robotics
         * and Automation, pages 1469-1474, 2001.
         *
         * http://dx.doi.org/10.1109/ROBOT.2001.932817
         */
        class WorkspaceSphereExplorer
        {
        public:
            enum Greedy
            {
                GREEDY_DISTANCE,
                GREEDY_SOURCE_DISTANCE,
                GREEDY_SPACE
            };

            WorkspaceSphereExplorer();

            OpenMesh::TriMesh_ArrayKernelT<> meshOpenMesh; //Added for the abbility of visualisation


            virtual ~WorkspaceSphereExplorer();

            bool explore();

            WorkspaceSphereList getPath() const;

            bool isCovered(const Vector3& point) const;

            void reset();

            void seed(const ::std::mt19937::result_type& value);

            Vector3* goal;

            Greedy greedy;

//                    DistanceModel* model;

            double radius;

            double range;

            ::std::size_t samples;

            Vector3* start;
            //Adding the FCL model for distance function
            std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> model;

        protected:

            struct VertexBundle
            {
                WorkspaceSphere sphere;
            };

            typedef ::boost::adjacency_list<
                    ::boost::listS,
                    ::boost::listS,
                    ::boost::bidirectionalS,
                    VertexBundle
            > Graph;

            typedef ::boost::graph_traits<Graph>::edge_descriptor Edge;

            typedef ::boost::graph_traits<Graph>::edge_iterator EdgeIterator;

            typedef ::std::pair<EdgeIterator, EdgeIterator> EdgeIteratorPair;

            typedef ::boost::graph_traits<Graph>::vertex_descriptor Vertex;

            typedef ::boost::graph_traits<Graph>::vertex_iterator VertexIterator;

            typedef ::std::pair<VertexIterator, VertexIterator> VertexIteratorPair;

            Edge addEdge(const Vertex& u, const Vertex& v);

            Vertex addVertex(const WorkspaceSphere& sphere);

            bool isCovered(const Vertex& parent, const Vector3& point) const;

            Sphere_aproximation Sphere; //For the visualisation

            Vertex begin;

            Vertex end;

            Graph graph;
            //multiset is an associative container that contains a sorted set of objects of type Key.
            ::std::multiset<WorkspaceSphere> queue;
            // variate_generator template<class Engine, class Distribution>
            ::boost::variate_generator< ::std::mt19937, ::boost::uniform_on_sphere< double>> rand;

        private:

        };
    }
}


#endif //BASE_OMPL_WORKSPACESPHEREEXPLORER_H
